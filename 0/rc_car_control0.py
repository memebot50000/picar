from gpiozero import Motor
import time

# Set up the motors
left_motor = Motor(forward=17, backward=27, enable=12)
right_motor = Motor(forward=22, backward=23)

def forward():
    left_motor.forward()
    right_motor.forward()

def backward():
    left_motor.backward()
    right_motor.backward()

def left():
    left_motor.backward()
    right_motor.forward()

def right():
    left_motor.forward()
    right_motor.backward()

def stop():
    left_motor.stop()
    right_motor.stop()

print("RC Car Control Ready. Enter W,A,S,D to control. Enter Q to quit.")

while True:
    key = input("Enter command: ").lower()
    if key == 'w':
        forward()
    elif key == 's':
        backward()
    elif key == 'a':
        left()
    elif key == 'd':
        right()
    elif key == 'q':
        stop()
        break
    else:
        stop()
    time.sleep(0.1)  # Short delay to prevent overwhelming the system

print("RC Car Control stopped.")

