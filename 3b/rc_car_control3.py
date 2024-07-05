from gpiozero import Motor
import time

# Set up the motors
drive_motor = Motor(forward=17, backward=27, enable=12)
steer_motor = Motor(forward=22, backward=23)

def forward(t):
    drive_motor.forward()
    time.sleep(t)
    drive_motor.stop()

def backward(t):
    drive_motor.backward()
    time.sleep(t)
    drive_motor.stop

def left(d, t):
    if d == "w":
        steer_motor.forward()
        forward(t)
        
        

def right():
    steer_motor.backward()

def stop():
    drive_motor.stop()
    steer_motor.stop()

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
