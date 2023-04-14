from subprocess import Popen, PIPE, run
import re, glob
from os import listdir, path, getcwd
import sox
from time import sleep
import gtts


def playDemo(demo, text):
    if demo:
        # make request to google to get synthesis
        tts = gtts.gTTS(text)
        # save the audio file
        tts.save("demo.mp3")
        # play the audio file
        run(["play", "demo.mp3"])

def get_audio(locale):
    all_filles = []
    # audio_folder = f"/Users/aretivykh/PycharmProjects/{locale}/-----"
    audio_folder = "/Users/aretivykh/PycharmProjects/Moto_Moto/"
    for file in glob.glob(path.join(audio_folder, '*.wav')):
    # for file in glob.glob(path.join(audio_folder, '*.mp3')):
        # all_filles.append(path.join(audio_folder, file))
        all_filles.append(file)
    return all_filles[:3]


def playWakeWord_to_HATS_Speaker(track, sound_device):
    """
    1 0 - Rirgh channel HATS
    0 1 - Left channel SPEAKER
    :param track:
    """
    # print(run(getcwd())
    # run(['cd enus-hf'])
    print(getcwd())

    if sound_device == 'HATS':
        # run(["play", track])
        Popen(f'play {track} remix 1 0' , shell=True)
        # run(['play',  f'{track} remix 1 0'])
    else:
        Popen(f'play {track} remix 0 1', shell=True)
        # run(["play hello.mp3 remix 0 1"])
        # run(['play', track])


# def playDemo(demo, track):
#     if demo:
#         run(["play", track])