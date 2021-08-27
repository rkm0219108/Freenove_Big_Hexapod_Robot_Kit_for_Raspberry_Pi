import time

from ADS7830 import *
from Buzzer import *
from Control import *
from Led import *
from Servo import *
from Ultrasonic import *

led = Led()


def test_led():
    try:
        # Red wipe
        print("\nRed wipe")
        led.color_wipe(Color(255, 0, 0))
        time.sleep(1)

        # Green wipe
        print("\nGreen wipe")
        led.color_wipe(Color(0, 255, 0))
        time.sleep(1)

        # Blue wipe
        print("\nBlue wipe")
        led.color_wipe(Color(0, 0, 255))
        time.sleep(1)

        # White wipe
        print("\nWhite wipe")
        led.color_wipe(Color(255, 255, 255))
        time.sleep(1)

        led.color_wipe(Color(0, 0, 0))  # turn off the light
        print("\nEnd of program")
    except KeyboardInterrupt:
        led.color_wipe(Color(0, 0, 0))  # turn off the light
        print("\nEnd of program")


ultrasonic = Ultrasonic()


def test_ultrasonic():
    try:
        while True:
            data = ultrasonic.get_distance()  # Get the value
            print("Obstacle distance is "+str(data)+"CM")
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nEnd of program")


servo = Servo()


def test_servo():
    try:
        for i in range(50):
            servo.set_servo_angle(15, 90+i)
            servo.set_servo_angle(12, 90+i)
            servo.set_servo_angle(9, 90+i)
            servo.set_servo_angle(16, 90+i)
            servo.set_servo_angle(19, 90+i)
            servo.set_servo_angle(22, 90+i)
            time.sleep(0.005)
        for i in range(60):
            servo.set_servo_angle(14, 90+i)
            servo.set_servo_angle(11, 90+i)
            servo.set_servo_angle(8, 90+i)
            servo.set_servo_angle(17, 90-i)
            servo.set_servo_angle(20, 90-i)
            servo.set_servo_angle(23, 90-i)
            time.sleep(0.005)
        for i in range(120):
            servo.set_servo_angle(13, i)
            servo.set_servo_angle(10, i)
            servo.set_servo_angle(31, i)
            servo.set_servo_angle(18, 180-i)
            servo.set_servo_angle(21, 180-i)
            servo.set_servo_angle(27, 180-i)
            time.sleep(0.005)
        print("\nEnd of program")
    except KeyboardInterrupt:
        print("\nEnd of program")


adc = ADS7830()


def test_adc():
    try:
        while True:
            power = adc.battery_power()
            print("The battery voltage is " + str(power) + '\n')
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nEnd of program")


buzzer = Buzzer()


def test_buzzer():
    try:
        buzzer.run('1')
        time.sleep(1)
        print("1S")
        time.sleep(1)
        print("2S")
        time.sleep(1)
        print("3S")
        buzzer.run('0')
        print("\nEnd of program")
    except KeyboardInterrupt:
        buzzer.run('0')
        print("\nEnd of program")


# Main program logic follows:


if __name__ == '__main__':
    print('Program is starting ... ')
    import sys
    if len(sys.argv) < 2:
        print("Parameter error: Please assign the device")
        exit()
    if sys.argv[1] == 'Led':
        test_led()
    elif sys.argv[1] == 'Ultrasonic':
        test_ultrasonic()
    elif sys.argv[1] == 'Servo':
        test_servo()
    elif sys.argv[1] == 'ADC':
        test_adc()
    elif sys.argv[1] == 'Buzzer':
        test_buzzer()
