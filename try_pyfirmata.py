# from pyfirmata import Arduino, util
# from time import sleep
#
#
# board = Arduino("/dev/cu.usbmodem14301")
#
# for x in range(500):
#     board.digital[2].write(1)
#     sleep(0.5)
#     board.digital[2].write(0)
#     sleep(0.5)


failed_angle = 100
TEST_STEPS = 5
test_points = list(range(failed_angle - TEST_STEPS * 3, failed_angle + (TEST_STEPS * 3)+ TEST_STEPS, 5))
test_points.remove(int(failed_angle))
print(failed_angle + (TEST_STEPS*3))
print(test_points)