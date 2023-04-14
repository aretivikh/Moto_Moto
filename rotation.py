import serial
import sys
from time import sleep


class Rotation():

    # def __init__(self, steps_per_revolution = 200, motor_id="/dev/ttyACM0"):
    def __init__(self):

        try:
            # set up serial connection to Arduino
            self.ser = serial.Serial('/dev/cu.usbmodem14301', 9600)
            self.current_position = 0
        except:
            print("No Arduino card")
            sys.exit()
            pass

    def read_serial(self):
        for i in range(50):
            line = self.ser.readline()  # read a byte
            if line:
                string = line.decode()  # convert the byte string to a unicode string
                num = int(string)  # convert the unicode string to an int
                print(num)

    def step(self, target_angle, command, home=False):
        self.target_angle = 0
        timeout_per_degree = 0.14
        timeout = 0

        if home:
            self.target_angle = -200 * self.current_position
        else:
            if self.current_position == 0:
                self.target_angle = target_angle * 200
                timeout = target_angle * timeout_per_degree
            elif self.current_position < target_angle:  # 70  100
                self.target_angle = 200 * (target_angle - self.current_position)
                timeout = (target_angle - self.current_position) * timeout_per_degree
            elif self.current_position > target_angle:  # 70 100
                self.target_angle = -200 * (self.current_position - target_angle)
                timeout = (self.current_position - target_angle) * timeout_per_degree
            elif self.current_position == target_angle:  # 180 == 180
                return
            else:
                return


        try:
            if command == 'cw':
                self.ser.write(b'cw ' + str(self.target_angle).encode() + b'\n')
                self.current_position = target_angle
                sleep(timeout)
            elif command == 'ccw':
                self.ser.write(b'ccw ' + str(self.target_angle).encode() + b'\n')
                self.current_position = self.target_angle
                sleep(timeout)
            elif command == 'r':
                # reset current position to 0
                self.ser.write(b'r ' + str(self.target_angle).encode() + b'\n')
                sleep(timeout)
            elif command == 'home':
                self.ser.write(b'r ' + str(self.target_angle).encode() + b'\n')
                sleep(timeout)
        except:
            self.ser = serial.Serial('/dev/cu.usbmodem14301', 9600)
            if command == 'cw':
                self.ser.write(b'cw ' + str(self.target_angle).encode() + b'\n')
                self.current_position = target_angle
                sleep(timeout)
            elif command == 'ccw':
                self.ser.write(b'ccw ' + str(self.target_angle).encode() + b'\n')
                self.current_position = self.target_angle
                sleep(timeout)
            elif command == 'r':
                # reset current position to 0
                self.ser.write(b'r ' + str(self.target_angle).encode() + b'\n')
                sleep(timeout)
            elif command == 'home':
                self.ser.write(b'r ' + str(self.target_angle).encode() + b'\n')
                sleep(timeout)



    def emeergency_stop(self):
        pass

    def close_connection(self):
        # close motor
        self.ser.close()