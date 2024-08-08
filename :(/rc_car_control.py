import serial
import cv2
import numpy as np
from picamera2 import Picamera2  # Changed from picamera to picamera2
from pymavlink import mavutil
import time
import RPi.GPIO as GPIO
import struct
import threading

# Set up GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Motor driver pins
DRIVE_ENA = 12
DRIVE_IN1 = 16
DRIVE_IN2 = 20
STEER_ENB = 13
STEER_IN3 = 19
STEER_IN4 = 26

# Set up motor driver pins
GPIO.setup(DRIVE_ENA, GPIO.OUT)
GPIO.setup(DRIVE_IN1, GPIO.OUT)
GPIO.setup(DRIVE_IN2, GPIO.OUT)
GPIO.setup(STEER_ENB, GPIO.OUT)
GPIO.setup(STEER_IN3, GPIO.OUT)
GPIO.setup(STEER_IN4, GPIO.OUT)

# Set up PWM for motor speed control
drive_pwm = GPIO.PWM(DRIVE_ENA, 1000)
steer_pwm = GPIO.PWM(STEER_ENB, 1000)
drive_pwm.start(0)
steer_pwm.start(0)

# RC receiver input pins
THROTTLE_PIN = 5
STEERING_PIN = 6

# Set up RC receiver input pins
GPIO.setup(THROTTLE_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(STEERING_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Set up serial connection to SP Racing MOF3 V1 via USB
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

# Set up camera
camera = Picamera2()  # Changed to Picamera2
camera.configure(camera.create_still_configuration(main={"size": (640, 480)}))
camera.start()

# Allow the camera to warm up
time.sleep(0.1)

# Set up MAVLink connection
mav_connection = mavutil.mavlink_connection('udpout:192.168.1.100:14550')

# Global variables
autonomous_mode = False
obstacle_detected = False

def msp_encode(cmd, data):
    total_data = [ord('$'), ord('M'), ord('<'), len(data), cmd] + data
    checksum = 0
    for i in total_data[3:]:
        checksum ^= i
    total_data.append(checksum)
    return bytes(total_data)

def msp_decode(data):
    if len(data) < 5:
        return None
    if data[0] != ord('$') or data[1] != ord('M') or data[2] != ord('>'):
        return None
    data_length = data[3]
    cmd = data[4]
    if len(data) < data_length + 6:
        return None
    payload = data[5:5+data_length]
    checksum = data[5+data_length]
    calculated_checksum = 0
    for i in data[3:5+data_length]:
        calculated_checksum ^= i
    if checksum != calculated_checksum:
        return None
    return cmd, payload

def read_imu_data():
    cmd = 102  # MSP_RAW_IMU
    encoded = msp_encode(cmd, [])
    ser.write(encoded)
    response = ser.read(21)  # 9 * 2 bytes of data + 3 header bytes
    decoded = msp_decode(response)
    if decoded:
        cmd, payload = decoded
        if cmd == 102 and len(payload) == 18:
            data = struct.unpack('<hhhhhhhhh', bytes(payload))
            accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z = data
            return {
                'accel': (accel_x, accel_y, accel_z),
                'gyro': (gyro_x, gyro_y, gyro_z),
                'mag': (mag_x, mag_y, mag_z)
            }
    return None

def capture_image():
    return camera.capture_array()  # Changed to use Picamera2 method

def process_image(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest_contour) > 5000:  # Adjust this threshold as needed
            return True
    return False

def fuse_sensor_data(imu_data, vision_data):
    return {'imu': imu_data, 'vision': vision_data}

def autonomous_control(fused_data):
    if fused_data['vision']:  # Example threshold for vision-based obstacle detection
        return {'drive': 0, 'steer': 0}
    return {'drive': 50, 'steer': 0}

def send_telemetry(mav_connection, data):
    if data['imu'] is not None:
        mav_connection.mav.attitude_send(
            int(time.time() * 1e6),
            data['imu']['gyro'][0],
            data['imu']['gyro'][1],
            data['imu']['gyro'][2],
            data['imu']['accel'][0],
            data['imu']['accel'][1],
            data['imu']['accel'][2]
        )

def handle_qgc_commands():
    global autonomous_mode, obstacle_detected
    while True:
        msg = mav_connection.recv_match(blocking=True)
        if msg is not None:
            if msg.get_type() == 'COMMAND_LONG':
                command = msg.command
                if command == mavutil.mavlink.MAV_CMD_NAV_GUIDED_ENABLE:
                    autonomous_mode = (msg.param1 == 1)
                    print(f"Autonomous mode: {'Enabled' if autonomous_mode else 'Disabled'}")

def control_motors(drive_speed, steer_speed):
    # Control drive motor
    if drive_speed > 0:
        GPIO.output(DRIVE_IN1, GPIO.HIGH)
        GPIO.output(DRIVE_IN2, GPIO.LOW)
        drive_pwm.ChangeDutyCycle(drive_speed)
    elif drive_speed < 0:
        GPIO.output(DRIVE_IN1, GPIO.LOW)
        GPIO.output(DRIVE_IN2, GPIO.HIGH)
        drive_pwm.ChangeDutyCycle(-drive_speed)
    else:
        GPIO.output(DRIVE_IN1, GPIO.LOW)
        GPIO.output(DRIVE_IN2, GPIO.LOW)
        drive_pwm.ChangeDutyCycle(0)

    # Control steering motor
    if steer_speed > 0:
        GPIO.output(STEER_IN3, GPIO.HIGH)
        GPIO.output(STEER_IN4, GPIO.LOW)
        steer_pwm.ChangeDutyCycle(steer_speed)
    elif steer_speed < 0:
        GPIO.output(STEER_IN3, GPIO.LOW)
        GPIO.output(STEER_IN4, GPIO.HIGH)
        steer_pwm.ChangeDutyCycle(-steer_speed)
    else:
        GPIO.output(STEER_IN3, GPIO.LOW)
        GPIO.output(STEER_IN4, GPIO.LOW)
        steer_pwm.ChangeDutyCycle(0)

def read_rc_input():
    throttle_pulse = GPIO.wait_for_edge(THROTTLE_PIN, GPIO.RISING, timeout=20)
    if throttle_pulse is not None:
        throttle_start = time.time()
        GPIO.wait_for_edge(THROTTLE_PIN, GPIO.FALLING, timeout=20)
        throttle_pulse_width = (time.time() - throttle_start) * 1000  # in ms

    steering_pulse = GPIO.wait_for_edge(STEERING_PIN, GPIO.RISING, timeout=20)
    if steering_pulse is not None:
        steering_start = time.time()
        GPIO.wait_for_edge(STEERING_PIN, GPIO.FALLING, timeout=20)
        steering_pulse_width = (time.time() - steering_start) * 1000  # in ms

    # Convert pulse width to motor speed (-100 to 100)
    throttle = max(min(int((throttle_pulse_width - 1500) / 5), 100), -100)
    steering = max(min(int((steering_pulse_width - 1500) / 5), 100), -100)

    return throttle, steering

# Start the QGC command handling in a separate thread
qgc_thread = threading.Thread(target=handle_qgc_commands)
qgc_thread.daemon = True
qgc_thread.start()

try:
    while True:
        imu_data = read_imu_data()
        image = capture_image()
        obstacle_detected = process_image(image)
        fused_data = fuse_sensor_data(imu_data, obstacle_detected)
        
        if autonomous_mode:
            control_commands = autonomous_control(fused_data)
            print(f"Autonomous control: {control_commands}")
            control_motors(control_commands['drive'], control_commands['steer'])
        else:
            # Manual control using RC receiver input
            throttle, steering = read_rc_input()
            print(f"Manual control: Throttle: {throttle}, Steering: {steering}")
            control_motors(throttle, steering)
        
        send_telemetry(mav_connection, fused_data)
        
        time.sleep(0.1)  # Adjust the sleep time as needed

except KeyboardInterrupt:
    print("Stopping...")
finally:
    drive_pwm.stop()
    steer_pwm.stop()
    GPIO.cleanup()
    camera.stop()  # Stop the camera when the script ends
