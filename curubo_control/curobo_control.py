#!/usr/bin/env python3
import torch
import numpy as np
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy, JointState as JointStateMsg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from tf_transformations import quaternion_from_euler, quaternion_multiply
from curobo.types.robot import RobotConfig, JointState
from curobo.types.math import Pose
from curobo.wrap.reacher.motion_gen import (
    MotionGen,
    MotionGenConfig,
    MotionGenPlanConfig,
)


class CuroboControlNode(Node):
    def __init__(self) -> None:
        super().__init__("curobo_control")
        self.joint_state_subscription_ = self.create_subscription(
            JointStateMsg,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        self.cmd_vel_subscription_ = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10,
        )

        self.joy_subscription_ = self.create_subscription(
            Joy,
            "/joy",
            self.joy_callback,
            10,
        )

        self.joint_trajectory_publisher = self.create_publisher(
            JointTrajectory, "/scaled_joint_trajectory_controller/joint_trajectory", 10
        )
        self.gripper_action_client = ActionClient(
            self, GripperCommand, "/robotiq_gripper_controller/gripper_cmd"
        )

        self.ur5e_cfg = RobotConfig.from_basic(
            Path(__file__).parent / "ur5e.urdf",
            "ur5e_base_link",
            "ur5e_tool0",
        )

        self.motion_gen_config = MotionGenConfig.load_from_robot_config(
            self.ur5e_cfg,
            tensor_args=self.ur5e_cfg.tensor_args,
            interpolation_dt=2.5,
            num_graph_seeds=1,
        )

        self.motion_gen = MotionGen(self.motion_gen_config)
        self.motion_gen.warmup()

        self.current_joint_state = None
        self.get_logger().info(
            "Finished loading cuda robot model, waiting for `/cmd_vel`..."
        )

    def joint_state_callback(self, msg: JointStateMsg) -> None:
        self.current_joint_state = msg

    GRIPPER_MAX = 40.0

    def joy_callback(self, msg: Joy) -> None:
        gripper_position = self.GRIPPER_MAX * ((float(msg.axes[5]) - 1) / -2)

        self._set_gripper_position(gripper_position)

        if bool(msg.buttons[0]):
            self._reset_to_home()

    def _set_gripper_position(self, position: float):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = 10.0

        self.gripper_action_client.wait_for_server()

        def noop(*args):
            pass

        send_goal_future = self.gripper_action_client.send_goal_async(
            goal_msg, feedback_callback=noop
        )
        send_goal_future.add_done_callback(noop)

    def _reset_to_home(self):
        trajectory_msg = JointTrajectory()
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        trajectory_msg.joint_names = [
            "ur5e_shoulder_pan_joint",
            "ur5e_shoulder_lift_joint",
            "ur5e_elbow_joint",
            "ur5e_wrist_1_joint",
            "ur5e_wrist_2_joint",
            "ur5e_wrist_3_joint",
        ]
        point = JointTrajectoryPoint()
        point.positions = [
            -np.pi / 2,
            -np.pi / 2,
            -np.pi / 2,
            -np.pi / 2,
            -(3 / 2) * np.pi,
            -(3 / 2) * np.pi,
        ]
        point.time_from_start = Duration(seconds=2.0).to_msg()
        trajectory_msg.points.append(point)

        self.trajectory_publisher_.publish(trajectory_msg)

    def _curobo_joint_state_to_ros(
        self, solver_joint_state: JointState
    ) -> JointStateMsg:
        ros_joint_state = JointStateMsg()
        ros_joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_names = getattr(solver_joint_state, "joint_names", None)
        # ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        joint_names = [
            "ur5e_shoulder_pan_joint",
            "ur5e_shoulder_lift_joint",
            "ur5e_elbow_joint",
            "ur5e_wrist_1_joint",
            "ur5e_wrist_2_joint",
            "ur5e_wrist_3_joint",
        ]

        if not joint_names and self.current_joint_state is not None:
            joint_names = self.current_joint_state.name
        ros_joint_state.name = list(joint_names) if joint_names is not None else []
        ros_joint_state.position = (
            solver_joint_state.position.detach().cpu().reshape(-1).tolist()
        )

        velocities = getattr(solver_joint_state, "velocity", None)
        if velocities is not None:
            ros_joint_state.velocity = velocities.detach().cpu().reshape(-1).tolist()

        effort = getattr(solver_joint_state, "torque", None)
        if effort is None:
            effort = getattr(solver_joint_state, "effort", None)
        if effort is not None:
            ros_joint_state.effort = effort.detach().cpu().reshape(-1).tolist()

        return ros_joint_state

    def cmd_vel_callback(self, msg: Twist):
        if self.current_joint_state is None:
            self.get_logger().warn("Skipped velocity command due to no joint state")
            return

        current_joint_state = [0] * 6
        for i, joint_name in enumerate(
            self.ur5e_cfg.kinematics.kinematics_config.joint_names
        ):
            idx = self.current_joint_state.name.index(joint_name)
            current_joint_state[i] = self.current_joint_state.position[idx]

        current_joint_state = torch.Tensor(current_joint_state).to(
            **self.ur5e_cfg.tensor_args.as_torch_dict()
        )

        assert current_joint_state.shape == (6,), f"{current_joint_state.shape=} =/= 6"

        end_effector_pose = self.motion_gen.kinematics.get_state(current_joint_state)

        current_position = end_effector_pose.ee_position[0]

        current_orientation = end_effector_pose.ee_quaternion[0]

        # Fix the end effector's orientation down with swing-twist decomposition
        down_vec = torch.Tensor([0, 1, 0]).to(
            **self.ur5e_cfg.tensor_args.as_torch_dict()
        )
        twist_vec = (
            torch.dot(down_vec, current_orientation[:3]) * current_orientation[:3]
        )

        twist_quat = torch.zeros_like(current_orientation)
        twist_quat[:3] = twist_vec[:]
        twist_quat[3] = current_orientation[3]
        twist_quat /= (twist_quat**2).sum()

        assert current_position.shape == (3,), f"{current_position.shape=} =/= 3"
        assert current_orientation.shape == (4,), f"{current_orientation.shape=} =/= 4"

        delta_position = torch.tensor(
            [msg.linear.x, msg.linear.y, msg.linear.z],
            device=current_position.device,
            dtype=current_position.dtype,
        )
        goal_position = current_position + delta_position

        delta_orientation_np = quaternion_from_euler(
            msg.angular.x,
            msg.angular.y,
            msg.angular.z,
        )
        current_orientation_np = current_orientation.detach().cpu().numpy()
        goal_orientation_np = quaternion_multiply(
            delta_orientation_np, current_orientation_np
        )

        goal_orientation_np = np.asarray(goal_orientation_np)
        goal_orientation = torch.from_numpy(goal_orientation_np).to(
            device=current_orientation.device,
            dtype=current_orientation.dtype,
        )

        goal_pose = Pose(
            position=goal_position.unsqueeze(0),
            quaternion=goal_orientation.unsqueeze(0),
        )

        start_state = JointState.from_position(
            current_joint_state.unsqueeze(0),
            joint_names=list(
                self.current_joint_state.name[: current_joint_state.shape[0]]
            ),
        )

        motion_gen_plan_config = MotionGenPlanConfig(
            max_attempts=2,
        )
        motion_plan = self.motion_gen.plan_single(
            start_state, goal_pose, motion_gen_plan_config
        )

        if not motion_plan.success:
            self.get_logger().warn(f"Motion planning failed")
            return

        target_joint_state = motion_plan.optimized_plan[-1]

        target_joint_state = self._curobo_joint_state_to_ros(target_joint_state)
        trajectory_msg = JointTrajectory()
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        trajectory_msg.joint_names = target_joint_state.name

        point = JointTrajectoryPoint()
        point.positions = target_joint_state.position
        # point.velocities = target_joint_state.velocity
        # point.effort = target_joint_state.effort
        point.time_from_start = Duration(seconds=0.5).to_msg()

        trajectory_msg.points.append(point)
        self.joint_trajectory_publisher.publish(trajectory_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointStateListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
