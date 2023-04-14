# from rotation_pymata4 import Rotation
from audio import get_audio, playWakeWord_to_HATS_Speaker, playDemo
from validator import validate_logcat
from time import sleep
from rotation import Rotation
import time


text_test_run = "Test run completed. Return HATS at the start position°"
text_test_completed = "Boundary test completed. Supernova detected wake word at posinion 130, 135 and 140°"
text5_1 = "I will try find boundary in range from 120° to 150° with step 5°"
text5 = "Test failed at 135°."
text4 = "Test is completed. Return HATS to 0°"
text3 = "Now I will rotate HATS and repeat test under different angles."
text2 = "Now I will start SSR test, side speech rejection. Speaker will play wake word at position 0°"
text1 = "the first test is accuracy, HATS will say wake word and supernova should recognize it. "

action = Rotation()
last_position = None

def run_test(angle, track, sound_device, home):
    if angle > 0:
        action.step(angle, 'cw', home)

    if angle == 0 and not forward:
        action.step(angle, 'cw', home)
    # action.read_serial()
    playWakeWord_to_HATS_Speaker(track, sound_device)
    sleep(3)
    # validate_logcat()

def runner(locale, angles):
    global last_position
    global demo
    global action
    global forward
    global boundary_demo
    boundary_demo = True
    forward = True
    demo = True
    home = False
    angles.sort()
    # angles.append(-1 * angles[-1]) #add negative maximum of array to find Home
    last_position = None
    # action.find_zero()
    testSet = get_audio(locale)
    for n, track in enumerate(testSet, 1):
        print(f'track # {n} of {len(testSet)}')
        if forward: # )0°>90°>180°>270°
            for angle in angles:
                print(f'forward {angle}')
                execute_test(action, angle, home, track, demo)
                if angle == 135:
                    playDemo(demo, text5)
                    boundary_test(135, track, boundary_demo)
                    playDemo(demo, text_test_completed)


            forward = False
            demo = False
        else:
            # It is done to start next test from the latest angle and don't rotate HATS
            # for example start from angle 270°>180>90>0
            for i in angles[::-1]:
                print(f'backward {i}')
                execute_test(action, i, home, track, demo)
            forward = True
        demo = False

        if testSet.index(track) == len(testSet)-1:
            playDemo(True, text_test_run)
            demo = False
            execute_test(action, 0, True, track, demo)

def boundary_test(failed_angle, track, boundary_demo, home=False):
    # global boundary_demo
    # in caase test failed at 135° try to find +/- boundary of ricky zone
    # step 5°, range 15° each direction, total 30°.      120< 125< 130< 135° > 140 > 145 > 150
    if boundary_demo:
        TEST_STEPS = 5
        test_points = list(range(failed_angle - TEST_STEPS * 3, failed_angle + (TEST_STEPS * 3)+ TEST_STEPS, 5))
        test_points.remove(int(failed_angle))
        text_ = f'I will check zone from f"{test_points[0]}° to {test_points[-1]}° with step 5°"'
        playDemo(demo, text_)
        for angle in test_points:
            playDemo(demo, f"check angle {angle}")
            run_test(angle, track, 'SPEAKER', home)
            print(angle)
        boundary_demo = False


def execute_test(action, angle, home, track, demo):
    global last_position
    last_position = angle
    if angle > 0:
        # test WW via HATS  should pass
        # playDemo(demo,f"Now I will rotate HATS and repeat test under {angle}°).")
        run_test(angle, track, 'SPEAKER', home)

    elif angle == 0:
        if  not forward:
            playDemo(demo, text2)
            run_test(0, track, 'SPEAKER', home)
        else:
            playDemo(demo, text1)
            run_test(angle, track, 'HATS', home)
            # rotate HATS to door and report
            # action.step(180, 'cw')
            # playDemo(demo,"I have detected wake word, test pass. Let me continue")
            # action.step(180, 'ccw')
            playDemo(demo, text2)
            run_test(0, track, 'SPEAKER', home)

        demo = False


if __name__ == '__main__':
    start=time.time()
    runner("en_us", [0, 90, 135, 180, 225, 270])
    # runner("en_us", [0, 90, 135, 180, 225, 270])
    # runner("en_us", [0, 90])
    action.close_connection()
    print(time.time() - start)
