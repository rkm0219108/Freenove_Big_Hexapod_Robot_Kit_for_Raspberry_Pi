import time

import RPi.GPIO as GPIO

TRIGGER_PIN = 27
ECHO_PIN = 22


class Ultrasonic:
    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(TRIGGER_PIN, GPIO.OUT)
        GPIO.setup(ECHO_PIN, GPIO.IN)

    def send_trigger_pulse(self):
        GPIO.output(TRIGGER_PIN, True)
        time.sleep(0.00015)
        GPIO.output(TRIGGER_PIN, False)

    def wait_for_echo(self, value: bool, timeout: int):
        count = timeout
        while GPIO.input(ECHO_PIN) != value and count > 0:
            count = count-1

    def get_distance(self):
        distance_cm: list[float] = []
        for i in range(3):
            self.send_trigger_pulse()
            self.wait_for_echo(True, 10000)
            start_time = time.time()
            self.wait_for_echo(False, 10000)
            finish_time = time.time()
            pulse_time = finish_time - start_time
            distance_cm.append(pulse_time / 0.000058)
        return int(min(distance_cm))


# Main program logic follows:
if __name__ == '__main__':
    sonic = Ultrasonic()
    while True:
        print(sonic.get_distance())
