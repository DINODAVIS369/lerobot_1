from setuptools import setup

package_name = 'so101_servo_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/servo.launch.py']),
        ('share/' + package_name + '/config', ['config/servo.yaml']),
    ],
    zip_safe=True,
    maintainer='dino',
    maintainer_email='dinodavis04885@gmail.com',
    description='Keyboard servo teleoperation for SO101 robot',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'keyboard_servo = so101_servo_teleop.keyboard_servo:main',
            'joint_state_relay = so101_servo_teleop.joint_state_relay:main', 
        ],
    },
)
