import sys
import termios
import time
import tty
import random


def read_char():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        char = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return char


def typed_vs_expected(text):
    for expected in text:
        typed = read_char()
        yield (typed, expected)


def color_char(typed, expected):
    color = '\033[92m' if typed == expected else '\033[91m'
    return f'{color}{typed}\033[0m'


def typing_speed(text, start_time, end_time):
    words = len(text) / 5
    minutes = (end_time - start_time) / 60
    return words / minutes


def display_typed(text):
    start_time = time.time()
    typed_list = []
    for typed, expected in typed_vs_expected(text):
        colored_char = color_char(typed, expected)
        print(colored_char, end='', flush=True)
        typed_list.append((typed, expected))
    end_time = time.time()
    elapsed_time = end_time - start_time
    return elapsed_time, typed_list


def accuracy(typed_list):
    correct = 0
    for typed, expected in typed_list:
        if typed == expected:
            correct += 1
    return correct / len(typed_list) * 100


def main():
    texts = [
        "The quick brown fox jumps over the lazy dog.",
        "The rain in Spain stays mainly in the plain.",
        "A gentleman is one who never hurts anyone's feelings unintentionally.",
        "People who think they know everything are a great annoyance to those of us who do.",
        "The world is a dangerous place to live; not because of the people who are evil, but because of the people who don't do anything about it."
    ]
    selected_text = random.choice(texts)
    print(selected_text)
    print()
    print()
    elapsed_time, typed_list = display_typed(selected_text)
    speed = typing_speed(selected_text, 0, elapsed_time)
    acc = accuracy(typed_list)
    print("\nYour typing speed is {:.2f} words per minute and your accuracy is {:.2f}%".format(speed, acc))


if __name__ == "__main__":
    main()