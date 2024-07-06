from gpiozero import Motor
import time

# Set up the motors
left_motor = Motor(forward=17, backward=27, enable=12)
right_motor = Motor(forward=22, backward=23, enable=13)

def stop():
    left_motor.stop()
    right_motor.stop()

def forward(t):
    '''drives car forward for t seconds'''
    left_motor.forward()
    right_motor.forward()
    time.sleep(t)
    stop()

def backward(t):
    '''drives car backward for t seconds'''
    left_motor.backward()
    right_motor.backward()
    time.sleep(t)
    stop()

def left(t):
    '''turns car left for t seconds'''
    left_motor.forward()
    right_motor.backward()
    time.sleep(t)
    stop()

def right(t):
    '''turns car right for t seconds'''
    left_motor.backward()
    right_motor.forward()
    time.sleep(t)
    stop()

def test():
    forward(0.5)
    backward(0.5)
    left(0.5)
    right(0.5)
    stop()
    print("test completed")

print("RC Car Control Ready. Enter W,A,S,D to control. Enter corresponding times after commands. Enter Q to quit. Enter T to test. Remember to put spaces between commands/times or the code will break.")

while True:
    kcmds = input("Enter command(s): ").lower()
    tcmds = input("Enter time(s): ")
    klst = kcmds.split()
    tlst = tcmds.split()
    tlst = [float(t) for t in tlst]  # Convert times to float
    
    try:
        for i in range(len(klst)):
            if klst[i] == 'w':
                forward(tlst[i])
            elif klst[i] == 's':
                backward(tlst[i])
            elif klst[i] == 'a':
                left(tlst[i])
            elif klst[i] == 'd':
                right(tlst[i])
                else:
                    stop()
            elif klst[i] == "t":
                test()
            elif klst[i] == 'q':
                stop()
                print("RC Car Control stopped.")
                exit()
            else:
                stop()
    except IndexError:
        print("Error: Number of commands and times do not match.")
    except ValueError:
        print("Error: Invalid time input. Please enter numeric values for times.")
    except Exception as e:
        print(f"An error occurred: {str(e)}")
    
    time.sleep(0.1)  # Short delay to prevent overwhelming the system
