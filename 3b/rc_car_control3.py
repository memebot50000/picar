from gpiozero import Motor
import time

# Set up the motors
drive_motor = Motor(forward=17, backward=27, enable=12)
steer_motor = Motor(forward=22, backward=23, enable=13)

def stop():
    drive_motor.stop()
    steer_motor.stop()

def forward(t):
    '''drives car forward for t seconds'''
    drive_motor.forward()
    time.sleep(t)
    drive_motor.stop()

def backward(t):
    '''drives car backward for t seconds'''
    drive_motor.backward()
    time.sleep(t)
    drive_motor.stop()

def left(d, t):
    '''turns left going d direction for t seconds. d must be "w" or "s"'''
    if d == "w":
        steer_motor.forward()
        forward(t)
        steer_motor.stop()
    elif d == "s":
        steer_motor.forward()
        backward(t)
        steer_motor.stop()
    else:
        stop()

def right(d, t):
    '''turns left going d direction for t seconds. d must be "w" or "s"'''
    if d == "w":
        steer_motor.backward()
        forward(t)
        steer_motor.stop()
    elif d == "s":
        steer_motor.forward()
        backward(t)
        steer_motor.stop()
    else:
        stop()
        

def test()
    forward(0.5)
    backward(0.5)
    steer_motor.forward()
    time.sleep(0.5)
    steer_motor.stop()
    steer_motor.backward()
    time.sleep(0.5)
    steer_motor.stop()
    print ("test completed")





print("RC Car Control Ready. Enter W,A,S,D to control. Enter corresponding times after commands. Enter Q to quit. Enter T to test. Remember to put spaces between commands/times or the code will break.")

while True:
    kcmds = str(input("Enter command(s): ").lower())
    tcmds = str(input("Enter time(s): ")
    klst = kcmds.split()
    tlst = tcmds.split()
    for i in range(len(tlst)):
        tlst[i] = int(tlst[i])
    try:
        for i in range(len(kcmds)):
            if kcmds[i] == 'w':
                forward(tlst[i])
            elif kcmds[i] == 's':
                backward(tlst[i])
            elif kcmds[i] == 'a':
                if kcmds[i-1] == "w":
                    left("w", tlst[i])
                elif kcmds[i-1] == "s"
                    left("s", tlst[i])
                else:
                    stop()
            elif kcmds[i] == 'd':
                if kcmds[i-1] == "w":
                    right("w", tlst[i])
                elif kcmds[i-1] == "s"
                    right("s", tlst[i])
                else:
                    stop()
            elif kcmds[i] == "t":
                test()
            elif kcmds[i] == 'q':
                stop()
                break
            else:
                stop()
        except:
            print("Invalid command input. Check command (or program) syntax and try again.")
    time.sleep(0.1)  # Short delay to prevent overwhelming the system

print("RC Car Control stopped.")
