import time

import RPi.GPIO as GPIO


BUZZER_PIN = 17


class Buzzer:
    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(BUZZER_PIN, GPIO.OUT)

    def run(self, command: str):
        if command != "0":
            GPIO.output(BUZZER_PIN, True)
        else:
            GPIO.output(BUZZER_PIN, False)


if __name__ == '__main__':
    B = Buzzer()
    B.run('1')
    time.sleep(3)
    B.run('0')
