#!/bin/bash

# Exit on any error
set -e

echo "Setting up ROS 2 and RC Car Control Package"

# Update the system
sudo apt update
sudo apt upgrade -y

# Set up the ROS 2 repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update

# Install ROS 2 Humble
sudo apt install -y ros-humble-desktop

# Install additional ROS 2 packages
sudo apt install -y python3-colcon-common-extensions ros-humble-teleop-twist-keyboard

# Set up ROS 2 environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Create a ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build

# Set up workspace environment
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Create RC car control package
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python rc_car_control

# Create setup.py file
cat << EOF > ~/ros2_ws/src/rc_car_control/setup.py
from setuptools import setup

package_name = 'rc_car_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS 2 package for RC car control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rc_car_control_node = rc_car_control.rc_car_control_node:main'
        ],
    },
)
EOF

# Create rc_car_control_node.py file
mkdir -p ~/ros2_ws/src/rc_car_control/rc_car_control
cat << EOF > ~/ros2_ws/src/rc_car_control/rc_car_control/rc_car_control_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
import time

class RCCarControlNode(Node):

    def __init__(self):
        super().__init__('rc_car_control_node')

        # Motor driver pins
        self.DRIVE_ENA = 12
        self.DRIVE_IN1 = 16
        self.DRIVE_IN2 = 20
        self.STEER_ENB = 13
        self.STEER_IN3 = 19
        self.STEER_IN4 = 26

        # Set up GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.DRIVE_ENA, GPIO.OUT)
        GPIO.setup(self.DRIVE_IN1, GPIO.OUT)
        GPIO.setup(self.DRIVE_IN2, GPIO.OUT)
        GPIO.setup(self.STEER_ENB, GPIO.OUT)
        GPIO.setup(self.STEER_IN3, GPIO.OUT)
        GPIO.setup(self.STEER_IN4, GPIO.OUT)

        # Set up PWM for motor speed control
        self.drive_pwm = GPIO.PWM(self.DRIVE_ENA, 1000)
        self.steer_pwm = GPIO.PWM(self.STEER_ENB, 1000)
        self.drive_pwm.start(0)
        self.steer_pwm.start(0)

        # Create subscribers
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(String, 'mode', self.mode_callback, 10)

        self.current_mode = 'manual'

    def cmd_vel_callback(self, msg):
        if self.current_mode == 'manual':
            self.control_motors(msg.linear.x, msg.angular.z)

    def mode_callback(self, msg):
        self.current_mode = msg.data
        self.get_logger().info(f'Switched to mode: {self.current_mode}')

    def control_motors(self, linear_x, angular_z):
        drive_speed = int(linear_x * 100)
        steer_speed = int(angular_z * 100)

        # Control drive motor
        if drive_speed > 0:
            GPIO.output(self.DRIVE_IN1, GPIO.HIGH)
            GPIO.output(self.DRIVE_IN2, GPIO.LOW)
            self.drive_pwm.ChangeDutyCycle(drive_speed)
        elif drive_speed < 0:
            GPIO.output(self.DRIVE_IN1, GPIO.LOW)
            GPIO.output(self.DRIVE_IN2, GPIO.HIGH)
            self.drive_pwm.ChangeDutyCycle(-drive_speed)
        else:
            GPIO.output(self.DRIVE_IN1, GPIO.LOW)
            GPIO.output(self.DRIVE_IN2, GPIO.LOW)
            self.drive_pwm.ChangeDutyCycle(0)

        # Control steering motor
        if steer_speed > 0:
            GPIO.output(self.STEER_IN3, GPIO.HIGH)
            GPIO.output(self.STEER_IN4, GPIO.LOW)
            self.steer_pwm.ChangeDutyCycle(steer_speed)
        elif steer_speed < 0:
            GPIO.output(self.STEER_IN3, GPIO.LOW)
            GPIO.output(self.STEER_IN4, GPIO.HIGH)
            self.steer_pwm.ChangeDutyCycle(-steer_speed)
        else:
            GPIO.output(self.STEER_IN3, GPIO.LOW)
            GPIO.output(self.STEER_IN4, GPIO.LOW)
            self.steer_pwm.ChangeDutyCycle(0)

def main(args=None):
    rclpy.init(args=args)
    node = RCCarControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()
EOF

# Update package.xml file
cat << EOF > ~/ros2_ws/src/rc_car_control/package.xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>rc_car_control</name>
  <version>0.0.0</version>
  <description>ROS 2 package for RC car control</description>
  <maintainer email="your_email@example.com">your_name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF

# Build the workspace
cd ~/ros2_ws
colcon build

# Install MAVROS
sudo apt-get install -y ros-humble-mavros ros-humble-mavros-extras

echo "Setup complete! Please reboot your system to ensure all changes take effect."
echo "After reboot, you can run the RC car control node with:"
echo "ros2 run rc_car_control rc_car_control_node"
echo "And launch MAVROS with:"
echo "ros2 launch mavros mavros.launch.py fcu_url:=udp://:14550@127.0.0.1:14550"
