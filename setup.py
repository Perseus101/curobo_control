from setuptools import find_packages, setup

package_name = 'curubo_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'tf-transformations',
    ],
    zip_safe=True,
    maintainer='camoore7',
    maintainer_email='camoore7@ncsu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'curobo_control = curubo_control.curobo_control:main',
            'joy_cmd_vel_node = curubo_control.joy_cmd_vel_node:main',
        ],
    },
)
