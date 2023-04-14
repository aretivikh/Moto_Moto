from subprocess import Popen, PIPE, run
import re


def validate_logcat():
    """
    :param x:
    :return:
    """
    regex = re.compile('(?<=SpeechControllerImpl: Speaking fragment text\s").*(?=" for)', re.U)

    # sleep(timeout)

    cmd = "adb logcat | grep 'Conf:' & sleep 3; adb shell killall -2 logcat "

    proc = Popen(f"{cmd}", stdout=PIPE, stderr=PIPE, shell=True)

    stringBank = ''
    for line in proc.stdout.readlines():
        result = regex.findall(str(line.decode("utf-8", "replace")))

        for i in result:
            if len(i) >= 1:
                stringBank += i + " "
            else:
                pass

    return True if stringBank else False
