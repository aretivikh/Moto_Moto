import serial
import time

# Define the serial port and baud rate
serial_port = '/dev/cu.usbmodem14301'




import time
import pyfirmata2



# Set up the Arduino board and stepper motor pins
PORT =  pyfirmata2.Arduino.AUTODETECT
board = pyfirmata2.Arduino(PORT)

# Define the step and direction pins for the stepper motor
step_pin = board.get_pin('d:2:o')
dir_pin = board.get_pin('d:3:o')

# Define the maximum speed and acceleration of the motor
max_speed = 200
acceleration = 100

# Create a function to move the motor by a specified number of steps
def move(steps):
    # Set the direction of the motor based on the sign of the steps argument
    if steps > 0:
        dir_pin.write(1)
    else:
        dir_pin.write(0)
    # Move the motor the specified number of steps
    for i in range(abs(steps)):
        step_pin.write(1)
        time.sleep(1/max_speed)
        step_pin.write(0)
        time.sleep(1/max_speed)

# Move the motor 1000 steps in one direction
move(1000)

# Move the motor 1000 steps in the other direction
move(-1000)

# Stop the motor by setting the step and direction pins to LOW
step_pin.write(0)
dir_pin.write(0)