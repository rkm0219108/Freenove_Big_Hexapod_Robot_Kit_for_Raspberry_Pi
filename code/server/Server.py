# -*- coding: utf-8 -*-
import fcntl
import io
import socket
import struct
import threading
import time

import picamera

from ADS7830 import *
from Buzzer import *
from Command import COMMAND as cmd
from Control import *
from Led import *
from Servo import *
from Thread import *
from Ultrasonic import *
from utils import restriction


class Server:
    def __init__(self):
        self.tcp_enabled = False
        self.led = Led()
        self.adc = ADS7830()
        self.servo = Servo()
        self.buzzer = Buzzer()
        self.control = Control()
        self.sonic = Ultrasonic()
        self.control.condition_thread.start()

    def get_interface_ip(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        return socket.inet_ntoa(fcntl.ioctl(s.fileno(), 0x8915, struct.pack('256s', b'wlan0'[:15]))[20:24])

    def turn_on_server(self):
        # ip adress
        HOST = self.get_interface_ip()
        # Port 8002 for video transmission
        self.video_socket = socket.socket()
        self.video_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.video_socket.bind((HOST, 8002))
        self.video_socket.listen(1)

        # Port 5002 is used for instruction sending and receiving
        self.instruction_socket = socket.socket()
        self.instruction_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.instruction_socket.bind((HOST, 5002))
        self.instruction_socket.listen(1)
        print('Server address: '+HOST)

    def turn_off_server(self):
        try:
            self.video_conn.close()
            self.instruction_conn.close()
        except:
            print('\n'+"No client connection")

    def reset_server(self):
        self.turn_off_server()
        self.turn_on_server()
        self.video = threading.Thread(target=self.transmission_video)
        self.instruction = threading.Thread(target=self.receive_instruction)
        self.video.start()
        self.instruction.start()

    def send_data(self, connect, data):
        try:
            connect.send(data.encode('utf-8'))
            # print("send",data)
        except Exception as e:
            print(e)

    def transmission_video(self):
        try:
            self.video_conn, self.video_address = self.video_socket.accept()
            self.video_conn = self.video_conn.makefile('wb')
        except:
            pass
        self.video_socket.close()
        try:
            with picamera.PiCamera() as camera:
                camera.resolution = (640, 480)      # pi camera resolution
                camera.framerate = 30               # 15 frames/sec
                camera.saturation = 80              # Set image video saturation
                # Set the brightness of the image (50 indicates the state of white balance)
                camera.brightness = 50
                #camera.iso = 400
                # give 2 secs for camera to initilize
                time.sleep(2)
                start = time.time()
                stream = io.BytesIO()
                # send jpeg format video stream
                print("Start transmit ... ")
                for foo in camera.capture_continuous(stream, 'jpeg', use_video_port=True):
                    try:
                        self.video_conn.flush()
                        stream.seek(0)
                        b = stream.read()
                        lengthBin = struct.pack('L', len(b))
                        self.video_conn.write(lengthBin)
                        self.video_conn.write(b)
                        stream.seek(0)
                        stream.truncate()
                    except BaseException as e:
                        #print (e)
                        print("End transmit ... ")
                        break
        except BaseException as e:
            # print(e)
            print("Camera unintall")

    def receive_instruction(self):
        try:
            self.instruction_conn, self.instruction_address = self.instruction_socket.accept()
            print("Client connection successful !")
        except:
            print("Client connect failed")
        self.instruction_socket.close()

        while True:
            try:
                all_data = self.instruction_conn.recv(1024).decode('utf-8')
            except:
                if self.tcp_enabled:
                    self.reset_server()
                    break
                else:
                    break
            if all_data == "" and self.tcp_enabled:
                self.reset_server()
                break
            else:
                cmd_array = all_data.split('\n')
                print(cmd_array)
                if cmd_array[-1] != "":
                    cmd_array == cmd_array[:-1]
            for one_cmd in cmd_array:
                data = one_cmd.split("#")
                if data == None or data[0] == '':
                    continue
                elif cmd.CMD_BUZZER in data:
                    self.buzzer.run(data[1])
                elif cmd.CMD_POWER in data:
                    batteryVoltage = self.adc.battery_power()
                    command = cmd.CMD_POWER+"#" + str(batteryVoltage[0])+"#"+str(batteryVoltage[1])+"\n"
                    # print(command)
                    self.send_data(self.instruction_conn, command)
                    if batteryVoltage[0] < 5.5 or batteryVoltage[1] < 6:
                        for i in range(3):
                            self.buzzer.run("1")
                            time.sleep(0.15)
                            self.buzzer.run("0")
                            time.sleep(0.1)
                elif cmd.CMD_LED in data:
                    try:
                        stop_thread(thread_led)
                    except:
                        pass
                    thread_led = threading.Thread(target=self.led.light, args=(data,))
                    thread_led.start()
                elif cmd.CMD_LED_MODE in data:
                    try:
                        stop_thread(thread_led)
                        # print("stop,yes")
                    except:
                        # print("stop,no")
                        pass
                    thread_led = threading.Thread(target=self.led.light, args=(data,))
                    thread_led.start()
                elif cmd.CMD_SONIC in data:
                    command = cmd.CMD_SONIC+"#" + str(self.sonic.get_distance())+"\n"
                    self.send_data(self.instruction_conn, command)
                elif cmd.CMD_HEAD in data:
                    if len(data) == 3:
                        self.servo.set_servo_angle(int(data[1]), int(data[2]))
                elif cmd.CMD_CAMERA in data:
                    if len(data) == 3:
                        x = restriction(int(data[1]), (50, 180))
                        y = restriction(int(data[2]), (0, 180))
                        self.servo.set_servo_angle(0, x)
                        self.servo.set_servo_angle(1, y)
                elif cmd.CMD_RELAX in data:
                    # print(data)
                    if self.control.relax_flag == False:
                        self.control.relax(True)
                        self.control.relax_flag = True
                    else:
                        self.control.relax(False)
                        self.control.relax_flag = False
                elif cmd.CMD_SERVO_POWER in data:
                    if data[1] == "0":
                        GPIO.output(GPIO_SERVO_POWER, True)
                    else:
                        GPIO.output(GPIO_SERVO_POWER, False)

                else:
                    self.control.order = data
                    self.control.timeout = time.time()
        try:
            stop_thread(thread_led)
        except:
            pass
        # try:
        #     stop_thread(thread_sonic)
        # except:
        #     pass
        print("close_recv")


if __name__ == '__main__':
    pass
