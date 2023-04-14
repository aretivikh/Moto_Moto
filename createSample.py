import serial
import time

# set up serial connection to Arduino
ser = serial.Serial('/dev/cu.usbmodem14301', 9600)  # replace with your serial port and baud rate

def step(target_angle, dir='cw'):
    if dir == 'CW':
        ser.write(b'cw ' + str(target_angle).encode() + b'\n')
    elif dir == 'CCW':
        ser.write(b'ccw ' + str(target_angle).encode() + b'\n')
    elif dir == 'R':
        # reset current position to 0
        ser.write(b'r ' + str(target_angle).encode() + b'\n')


# take 10 steps in each direction, pausing briefly between each step
for i in range(10):
    print('CW')
    step(8000, 'CW')
    # time.sleep(0.1)

# set_direction(0)

for i in range(10):
    print('CCW')
    step(8000, 'CCW')
    time.sleep(0.1)

# stop the motor
# set_speed(0)

# close the serial connection
ser.close()