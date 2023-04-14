from time import sleep
from PyMata.pymata import PyMata
from pymata4 import pymata4


# PyMata.play_tone()
class Rotation():

    # def __init__(self, steps_per_revolution = 200, motor_id="/dev/ttyACM0"):
    def __init__(self):

        self.steps_counter = 0
        self.current_angle = 0
        self.steps_per_revolution = 200  #steps, 0 -microsteps
        self.micro_steps = 0
        # self.one_degree = self.step_per_revolution / 200
        self.one_degree = 360 / self.steps_per_revolution
        self.max_position = 250 # 1 rev /180
        self.speed = self.steps_per_revolution / 4
        self.last_position = 0
        try:
            self.board = pymata4.Pymata4()
            # configure the stepper to use pins 9,10,11,12 and specify steps / revolution
            self.board.set_pin_mode_stepper(self.steps_per_revolution, [2, 3])
            sleep(1) # allow time for command and reply to go across the serial link

            # activate Home
            self.HOME = 0
            self.board.set_pin_mode_analog_input(self.HOME)

            # activate Limit
            self.LIMIT_SWITCH_PIN = 1
            self.board.set_pin_mode_analog_input(self.LIMIT_SWITCH_PIN)

        except:

            sleep(3)
            try:
                self.board = pymata4.Pymata4()
                # configure the stepper to use pins 9,10,11,12 and specify steps / revolution
                self.board.set_pin_mode_stepper(self.steps_per_revolution, [2, 3])
                sleep(1)  # allow time for command and reply to go across the serial link

                # activate Home
                self.HOME = 0
                self.board.set_pin_mode_analog_input(self.HOME)

                # activate Limit
                self.LIMIT_SWITCH_PIN = 1
                self.board.set_pin_mode_analog_input(self.LIMIT_SWITCH_PIN)
            except:
                pass




    def calculate_rotation_time(self, angle):
        # 360 / 800 steps = 0.45° per step
        # speed = 900 rpm 60 = 15 rev/sec
        #
        one_degre_time = (self.speed / 60 / 60) / 360 # 360°
        delay = angle * one_degre_time
        print(f"Wait time {delay}")
        return delay


    def rotate_motor(self, move_to_angle, speed=20):
        # move motor #0 500 steps forward at a speed of 20
        print(f'Rotate from {self.last_position} to {move_to_angle}' )
        # 1 rev stepper(360°) = 1° table
        #
        revolutions = move_to_angle - self.last_position
        rotatation_steps = 0
        if move_to_angle >0:
            for i in range(revolutions):
                # while True:
                    value, time = self.board.analog_read(self.LIMIT_SWITCH_PIN)
                    # print(time, value)
                    print(f'i = {i}  revolutions {revolutions}   steps {self.steps_per_revolution * 4}')
                    self.board.stepper_write(int(self.speed)*20, -1 * self.steps_per_revolution * 4)  # send step command
                    # self.board.stepper_write(int(self.speed)*2, -1 * self.steps_per_revolution * 4)  # send step command
                    self.last_position = move_to_angle
                    self.steps_counter += rotatation_steps
                    sleep(0.1)
            sleep(3)
        else:
            for i in range(abs(move_to_angle)):
                # value, time = self.board.analog_read(self.LIMIT_SWITCH_PIN)
                # print('value ', value)
                # if value < 20:
                #     limit = True
                #     break
                # else:
                    print(f'i = {i}  revolutions {revolutions} , steps {self.steps_per_revolution * 4}')
                    self.board.stepper_write(int(self.speed) * 20, self.steps_per_revolution * 8)  # send step command
                    # self.board.stepper_write(int(self.speed), rotatation_steps * 4)  # send step command
                    self.last_position = 0
                    self.steps_counter = 0
                    sleep(0.3)

    #


    def rotate_motor_works(self, move_to_angle, speed=20):
        # move motor #0 500 steps forward at a speed of 20
        print(f'Rotate to {move_to_angle}')
        rotatation_steps = 0
        if move_to_angle >0:
            rotatation_steps = int(self.steps_per_revolution * (move_to_angle - self.last_position) / 360)
            self.board.stepper_write(int(self.speed), -1 * rotatation_steps * 4)  # send step command
            self.last_position = move_to_angle
            self.steps_counter += rotatation_steps
            sleep(3)
        else:
            rotatation_steps = int(self.steps_per_revolution * abs(move_to_angle ) / 360)
            self.board.stepper_write(int(self.speed), self.steps_counter * 4)  # send step command
            # self.board.stepper_write(int(self.speed), rotatation_steps * 4)  # send step command
            print(f'counter {self.steps_counter}, to Home {rotatation_steps}')
            self.last_position = 0
            self.steps_counter = 0
            sleep(3)


action = Rotation()
last_position = None

action.rotate_motor(-100)
action.board.send_reset()