To use the setup script:

1. Save the script as `setup_rc_car_ros2.sh` in your home directory.
2. Make the script executable:
   ```bash
   chmod +x setup_rc_car_ros2.sh
   ```
3. Run the script:
   ```bash
   ./setup_rc_car_ros2.sh
   ```

This script will:
- Update your system
- Install ROS 2 Humble
- Set up a ROS 2 workspace
- Create and set up the RC car control package
- Install MAVROS for communication with QGroundControl

After running the script, reboot your system to ensure all changes take effect. Then you can run the RC car control node and MAVROS as indicated in the final echo statements of the script.

### Hardware Connection Instructions

1. **RC Receiver**:
   - Connect the throttle channel to GPIO 5 (Pin 29)
   - Connect the steering channel to GPIO 6 (Pin 31)
   - Connect the receiver's ground to a GND pin on the Raspberry Pi (Pin 6 or 9)
   - Connect the receiver's power to the 5V pin on the Raspberry Pi (Pin 2 or 4)

2. **L298N Motor Driver**:
   - ENA (Drive Motor Enable) -> GPIO 12 (Pin 32)
   - IN1 (Drive Motor Direction) -> GPIO 16 (Pin 36)
   - IN2 (Drive Motor Direction) -> GPIO 20 (Pin 38)
   - ENB (Steering Motor Enable) -> GPIO 13 (Pin 33)
   - IN3 (Steering Motor Direction) -> GPIO 19 (Pin 35)
   - IN4 (Steering Motor Direction) -> GPIO 26 (Pin 37)
   - Connect the motor driver's ground to a GND pin on the Raspberry Pi
   - Connect the motor driver's logic power (usually 5V) to the 5V pin on the Raspberry Pi
   - Connect the motor driver's motor power to your 12V battery

3. **Motors**:
   - Connect the drive motor to the motor driver's output A
   - Connect the steering motor to the motor driver's output B

4. **Flight Controller** (for telemetry and GPS, if using):
   - Connect the flight controller to the Raspberry Pi via USB
   - If using UART instead of USB:
     - FC's UART TX -> Raspberry Pi's RX (GPIO 15, Pin 10)
     - FC's UART RX -> Raspberry Pi's TX (GPIO 14, Pin 8)

5. **Camera** (if using for computer vision):
   - Connect the Raspberry Pi Camera Module to the camera port on the Raspberry Pi

### Using the System

After setting up the hardware and running the installation script, follow these steps to use the system:

1. **Start the RC Car Control Node**:
   Open a terminal and run:
   ```bash
   ros2 run rc_car_control rc_car_control_node
   ```

2. **Start MAVROS** (for communication with QGroundControl):
   In another terminal, run:
   ```bash
   ros2 launch mavros mavros.launch.py fcu_url:=udp://:14550@127.0.0.1:14550
   ```

3. **Set up QGroundControl**:
   - Open QGroundControl on your computer
   - Go to Application Settings > Comm Links
   - Add a new link with Type: UDP, and enter the Raspberry Pi's IP address and port 14550
   - Connect to this link

4. **Manual Control**:
   - The RC receiver inputs will be read by the `rc_car_control_node`
   - Move the throttle and steering sticks on your RC transmitter to control the car

5. **Switching Modes**:
   To switch between manual and autonomous modes, publish a message to the `/mode` topic:
   ```bash
   ros2 topic pub /mode std_msgs/msg/String "data: autonomous"
   ```
   Or back to manual:
   ```bash
   ros2 topic pub /mode std_msgs/msg/String "data: manual"
   ```

6. **Autonomous Control**:
   When in autonomous mode, you can send velocity commands:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
   ```
   This command will move the car forward at half speed and turn slightly to the right.

7. **Mission Planning with QGroundControl**:
   - In QGC, switch to the Plan view
   - Create a mission by adding waypoints on the map
   - Upload the mission to the vehicle

8. **Executing Missions**:
   - Implement a mission execution node that listens for mission commands from MAVROS and publishes appropriate `/cmd_vel` messages
   - This would require additional development to interpret mission commands and translate them into movement commands for the RC car

9. **Monitoring**:
   - Use QGroundControl to monitor the vehicle's status, including position (if GPS is connected), battery level, and other telemetry data
   - You can also use ROS 2 tools to monitor topics:
     ```bash
     ros2 topic echo /cmd_vel
     ros2 topic echo /mode
     ```

10. **Stopping the System**:
    - To stop the RC car, switch back to manual mode and center the RC transmitter sticks
    - To shut down the ROS 2 nodes, use Ctrl+C in the terminal windows running the nodes

Remember to always have a way to quickly stop the car in case of unexpected behavior, such as an emergency stop switch or the ability to quickly switch to manual control.

For safety:
- Always test in a controlled environment
- Start with low speeds when testing autonomous functions
- Be prepared to take manual control at any time
- Ensure all connections are secure before operating the vehicle

As you develop the system further, you might want to add features like obstacle detection, more advanced path planning, and better integration with QGroundControl for mission execution.
