# -*- coding: utf-8 -*-
import copy
import math
import threading
import time
from typing import List

import numpy as np
import RPi.GPIO as GPIO

from Command import COMMAND as cmd
from IMU import *
from PID import *
from Servo import *
from utils import map_num, restriction

GPIO_SERVO_POWER = 4


class Control:
    def __init__(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(GPIO_SERVO_POWER, GPIO.OUT)
        GPIO.output(GPIO_SERVO_POWER, False)
        self.imu = IMU()
        self.servo = Servo()
        self.move_flag = 0x01
        self.relax_flag = False
        self.pid = IncrementalPID(0.500, 0.00, 0.0025)
        self.flag = 0x00
        self.timeout = 0
        self.height = -25
        self.body_point: list[list[float]] = [
            [137.1, 189.4, self.height], 
            [225, 0, self.height], 
            [137.1, -189.4, self.height],
            [-137.1, -189.4, self.height], 
            [-225, 0, self.height], 
            [-137.1, 189.4, self.height]
        ]
        self.calibration_leg_point = self.read_from_txt('point')
        self.leg_point = [
            [140, 0, 0],
            [140, 0, 0],
            [140, 0, 0],
            [140, 0, 0],
            [140, 0, 0],
            [140, 0, 0]
        ]
        self.calibration_angle = [
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0]
        ]
        self.angle = [
            [90, 0, 0],
            [90, 0, 0],
            [90, 0, 0],
            [90, 0, 0],
            [90, 0, 0],
            [90, 0, 0]
        ]
        self.order = ['', '', '', '', '', '']
        self.calibration()
        self.set_leg_angle()
        self.condition_thread = threading.Thread(target=self.condition)

    def read_from_txt(self, filename: str):
        txt_file = open(filename + ".txt", "r")
        list_row = txt_file.readlines()
        list_source: list[list[str]] = []
        list_int = [[0] * 3] * 6
        for i in range(len(list_row)):
            column_list = list_row[i].strip().split("\t")
            list_source.append(column_list)
        for i in range(len(list_source)):
            for j in range(len(list_source[i])):
                list_int[i][j] = int(list_source[i][j])
        txt_file.close()
        return list_int

    def save_to_txt(self, list: List[List[int]], filename: str):
        txt_file = open(filename + '.txt', 'w')
        for i in range(len(list)):
            for j in range(len(list[i])):
                txt_file.write(str(list[i][j]))
                txt_file.write('\t')
            txt_file.write('\n')
        txt_file.close()

    def coordinate_to_angle(self, coordinate: List[int], l1: int=33, l2: int=90, l3: int=110):
        ox, oy, oz = coordinate
        a = math.pi/2-math.atan2(oz, oy)
        x_3 = 0
        x_4 = l1*math.sin(a)
        x_5 = l1*math.cos(a)
        l23 = math.sqrt((oz-x_5)**2+(oy-x_4)**2+(ox-x_3)**2)
        w = restriction((ox-x_3)/l23, (-1, 1))
        v = restriction((l2*l2+l23*l23-l3*l3)/(2*l2*l23), (-1, 1))
        u = restriction((l2**2+l3**2-l23**2)/(2*l3*l2), (-1, 1))
        b = math.asin(round(w, 2))-math.acos(round(v, 2))
        c = math.pi-math.acos(round(u, 2))
        a = round(math.degrees(a))
        b = round(math.degrees(b))
        c = round(math.degrees(c))
        return [a, b, c]

    def angle_to_coordinate(self, angle: List[int], l1: int=33, l2: int=90, l3: int=110):
        a, b, c = angle
        a = math.pi/180*a
        b = math.pi/180*b
        c = math.pi/180*c
        ox = round(l3*math.sin(b+c)+l2*math.sin(b))
        oy = round(l3*math.sin(a)*math.cos(b+c)+l2 * math.sin(a)*math.cos(b)+l1*math.sin(a))
        oz = round(l3*math.cos(a)*math.cos(b+c)+l2 * math.cos(a)*math.cos(b)+l1*math.cos(a))
        return [ox, oy, oz]

    def leg_point_to_coordinate(self, point: List[int]):
        ox, oy, oz = point
        return [-oz, ox, oy]

    def calibration(self):
        self.leg_point = [
            [140, 0, 0],
            [140, 0, 0],
            [140, 0, 0],
            [140, 0, 0],
            [140, 0, 0],
            [140, 0, 0]
        ]
        for i in range(6):
            self.calibration_angle[i] = self.coordinate_to_angle(self.leg_point_to_coordinate(self.calibration_leg_point[i]))
        for i in range(6):
            self.angle[i] = self.coordinate_to_angle(self.leg_point_to_coordinate(self.leg_point[i]))

        for i in range(6):
            self.calibration_angle[i][0] = self.calibration_angle[i][0] - self.angle[i][0]
            self.calibration_angle[i][1] = self.calibration_angle[i][1] - self.angle[i][1]
            self.calibration_angle[i][2] = self.calibration_angle[i][2] - self.angle[i][2]

    def set_leg_angle(self):
        if self.check_point():
            angle_range = (0, 180)
            for i in range(6):
                self.angle[i] = self.coordinate_to_angle(self.leg_point_to_coordinate(self.leg_point[i]))
            for i in range(3):
                self.angle[i][0] = restriction(self.angle[i][0]+self.calibration_angle[i][0], angle_range)
                self.angle[i][1] = restriction(90-(self.angle[i][1]+self.calibration_angle[i][1]), angle_range)
                self.angle[i][2] = restriction(self.angle[i][2]+self.calibration_angle[i][2], angle_range)
                self.angle[i+3][0] = restriction(self.angle[i+3][0]+self.calibration_angle[i+3][0], angle_range)
                self.angle[i+3][1] = restriction(90+self.angle[i+3][1]+self.calibration_angle[i+3][1], angle_range)
                self.angle[i+3][2] = restriction(180-(self.angle[i+3][2]+self.calibration_angle[i+3][2]), angle_range)

            # leg1
            self.servo.set_servo_angle(15, self.angle[0][0])
            self.servo.set_servo_angle(14, self.angle[0][1])
            self.servo.set_servo_angle(13, self.angle[0][2])

            # leg2
            self.servo.set_servo_angle(12, self.angle[1][0])
            self.servo.set_servo_angle(11, self.angle[1][1])
            self.servo.set_servo_angle(10, self.angle[1][2])

            # leg3
            self.servo.set_servo_angle(9, self.angle[2][0])
            self.servo.set_servo_angle(8, self.angle[2][1])
            self.servo.set_servo_angle(31, self.angle[2][2])

            # leg6
            self.servo.set_servo_angle(16, self.angle[5][0])
            self.servo.set_servo_angle(17, self.angle[5][1])
            self.servo.set_servo_angle(18, self.angle[5][2])

            # leg5
            self.servo.set_servo_angle(19, self.angle[4][0])
            self.servo.set_servo_angle(20, self.angle[4][1])
            self.servo.set_servo_angle(21, self.angle[4][2])

            # leg4
            self.servo.set_servo_angle(22, self.angle[3][0])
            self.servo.set_servo_angle(23, self.angle[3][1])
            self.servo.set_servo_angle(27, self.angle[3][2])
        else:
            print("This coordinate point is out of the active range")

    def check_point(self):
        result = True
        for i in range(6):
            leg_lenght = math.sqrt(self.leg_point[i][0]**2+self.leg_point[i][1]**2+self.leg_point[i][2]**2)
            if leg_lenght > 248 or leg_lenght < 90:
                result = False
                break
        return result

    def condition(self):
        while True:
            if (time.time()-self.timeout) > 10 and self.timeout != 0 and self.order[0] == '':
                self.timeout = time.time()
                self.relax(True)
                self.flag = 0x00
            if cmd.CMD_POSITION in self.order and len(self.order) == 4:
                if self.flag != 0x01:
                    self.relax(False)
                x = restriction(int(self.order[1]), (-40, 40))
                y = restriction(int(self.order[2]), (-40, 40))
                z = restriction(int(self.order[3]), (-20, 20))
                self.position(x, y, z)
                self.flag = 0x01
                self.order = ['', '', '', '', '', '']
            elif cmd.CMD_ATTITUDE in self.order and len(self.order) == 4:
                if self.flag != 0x02:
                    self.relax(False)
                r = restriction(int(self.order[1]), (-15, 15))
                p = restriction(int(self.order[2]), (-15, 15))
                y = restriction(int(self.order[3]), (-15, 15))
                point = self.posture_balance(r, p, y)
                self.coordinate_transformation(point)
                self.set_leg_angle()
                self.flag = 0x02
                self.order = ['', '', '', '', '', '']
            elif cmd.CMD_MOVE in self.order and len(self.order) == 6:
                if self.order[2] == "0" and self.order[3] == "0":
                    self.run(self.order)
                    self.order = ['', '', '', '', '', '']
                else:
                    if self.flag != 0x03:
                        self.relax(False)
                    self.run(self.order)
                    self.flag = 0x03
            elif cmd.CMD_BALANCE in self.order and len(self.order) == 2:
                if self.order[1] == "1":
                    self.order = ['', '', '', '', '', '']
                    if self.flag != 0x04:
                        self.relax(False)
                    self.flag = 0x04
                    self.imu6050()
            elif cmd.CMD_CALIBRATION in self.order:
                self.timeout = 0
                self.calibration()
                self.set_leg_angle()
                if len(self.order) >= 2:
                    if self.order[1] == "one":
                        self.calibration_leg_point[0][0] = int(self.order[2])
                        self.calibration_leg_point[0][1] = int(self.order[3])
                        self.calibration_leg_point[0][2] = int(self.order[4])
                        self.calibration()
                        self.set_leg_angle()
                    elif self.order[1] == "two":
                        self.calibration_leg_point[1][0] = int(self.order[2])
                        self.calibration_leg_point[1][1] = int(self.order[3])
                        self.calibration_leg_point[1][2] = int(self.order[4])
                        self.calibration()
                        self.set_leg_angle()
                    elif self.order[1] == "three":
                        self.calibration_leg_point[2][0] = int(self.order[2])
                        self.calibration_leg_point[2][1] = int(self.order[3])
                        self.calibration_leg_point[2][2] = int(self.order[4])
                        self.calibration()
                        self.set_leg_angle()
                    elif self.order[1] == "four":
                        self.calibration_leg_point[3][0] = int(self.order[2])
                        self.calibration_leg_point[3][1] = int(self.order[3])
                        self.calibration_leg_point[3][2] = int(self.order[4])
                        self.calibration()
                        self.set_leg_angle()
                    elif self.order[1] == "five":
                        self.calibration_leg_point[4][0] = int(self.order[2])
                        self.calibration_leg_point[4][1] = int(self.order[3])
                        self.calibration_leg_point[4][2] = int(self.order[4])
                        self.calibration()
                        self.set_leg_angle()
                    elif self.order[1] == "six":
                        self.calibration_leg_point[5][0] = int(self.order[2])
                        self.calibration_leg_point[5][1] = int(self.order[3])
                        self.calibration_leg_point[5][2] = int(self.order[4])
                        self.calibration()
                        self.set_leg_angle()
                    elif self.order[1] == "save":
                        self.save_to_txt(self.calibration_leg_point, 'point')
                self.order = ['', '', '', '', '', '']

    def relax(self, flag: bool):
        if flag:
            self.servo.relax()
        else:
            self.set_leg_angle()

    def coordinate_transformation(self, point):
        # leg1
        self.leg_point[0][0] = point[0][0] * math.cos(54/180*math.pi)+point[0][1]*math.sin(54/180*math.pi)-94
        self.leg_point[0][1] = -point[0][0] * math.sin(54/180*math.pi)+point[0][1]*math.cos(54/180*math.pi)
        self.leg_point[0][2] = point[0][2]-14
        # leg2
        self.leg_point[1][0] = point[1][0] * math.cos(0/180*math.pi)+point[1][1]*math.sin(0/180*math.pi)-85
        self.leg_point[1][1] = -point[1][0] * math.sin(0/180*math.pi)+point[1][1]*math.cos(0/180*math.pi)
        self.leg_point[1][2] = point[1][2]-14
        # leg3
        self.leg_point[2][0] = point[2][0] * math.cos(-54/180*math.pi)+point[2][1]*math.sin(-54/180*math.pi)-94
        self.leg_point[2][1] = -point[2][0] * math.sin(-54/180*math.pi)+point[2][1]*math.cos(-54/180*math.pi)
        self.leg_point[2][2] = point[2][2]-14
        # leg4
        self.leg_point[3][0] = point[3][0] * math.cos(-126/180*math.pi) + point[3][1]*math.sin(-126/180*math.pi)-94
        self.leg_point[3][1] = -point[3][0] * math.sin(-126/180*math.pi)+point[3][1]*math.cos(-126/180*math.pi)
        self.leg_point[3][2] = point[3][2]-14
        # leg5
        self.leg_point[4][0] = point[4][0] * math.cos(180/180*math.pi)+point[4][1]*math.sin(180/180*math.pi)-85
        self.leg_point[4][1] = -point[4][0] * math.sin(180/180*math.pi)+point[4][1]*math.cos(180/180*math.pi)
        self.leg_point[4][2] = point[4][2]-14
        # leg6
        self.leg_point[5][0] = point[5][0] * math.cos(126/180*math.pi)+point[5][1]*math.sin(126/180*math.pi)-94
        self.leg_point[5][1] = -point[5][0] * math.sin(126/180*math.pi)+point[5][1]*math.cos(126/180*math.pi)
        self.leg_point[5][2] = point[5][2]-14

    def position(self, x, y, z):
        point = copy.deepcopy(self.body_point)
        for i in range(6):
            point[i][0] = self.body_point[i][0]-x
            point[i][1] = self.body_point[i][1]-y
            point[i][2] = -30-z
            self.height = point[i][2]
            self.body_point[i][2] = point[i][2]
        self.coordinate_transformation(point)
        self.set_leg_angle()

    def posture_balance(self, r, p, y):
        pos = np.mat([0.0, 0.0, self.height]).T
        rpy = np.array([r, p, y]) * math.pi / 180
        R, P, Y = rpy[0], rpy[1], rpy[2]
        rotx = np.mat([
            [1, 0, 0],
            [0, math.cos(P), -math.sin(P)],
            [0, math.sin(P), math.cos(P)]
        ])
        roty = np.mat([
            [math.cos(R), 0, -math.sin(R)],
            [0, 1, 0],
            [math.sin(R), 0, math.cos(R)]
        ])
        rotz = np.mat([
            [math.cos(Y), -math.sin(Y), 0],
            [math.sin(Y), math.cos(Y), 0],
            [0, 0, 1]
        ])

        rot_mat = rotx * roty * rotz

        body_struc = np.mat([
            [55, 76, 0],
            [85, 0, 0],
            [55, -76, 0],
            [-55, -76, 0],
            [-85, 0, 0],
            [-55, 76, 0]
        ]).T

        footpoint_struc = np.mat([
            [137.1, 189.4, 0],
            [225, 0, 0],
            [137.1, -189.4, 0],
            [-137.1, -189.4, 0],
            [-225, 0, 0],
            [-137.1, 189.4, 0]
        ]).T

        AB = np.mat(np.zeros((3, 6)))
        ab = [[0] * 3] * 6
        for i in range(6):
            AB[:, i] = pos + rot_mat * footpoint_struc[:, i]
            ab[i][0] = AB[0, i]
            ab[i][1] = AB[1, i]
            ab[i][2] = AB[2, i]
        return (ab)

    def imu6050(self):
        point = self.posture_balance(0, 0, 0)
        self.coordinate_transformation(point)
        self.set_leg_angle()
        time.sleep(2)
        self.imu.accel_average_data, self.imu.gyro_average_data = self.imu.average_filter()
        time.sleep(1)
        while True:
            if self.order[0] != "":
                break
            time.sleep(0.02)
            roll, pitch, yaw = self.imu.imu_update()
            # roll=restriction(self.pid.PID_compute(roll),(-15,15))
            # pitch=restriction(self.pid.PID_compute(pitch),(-15,15))
            roll = self.pid.compute_pid(roll)
            pitch = self.pid.compute_pid(pitch)
            point = self.posture_balance(roll, pitch, 0)
            self.coordinate_transformation(point)
            self.set_leg_angle()

    # example : data=['CMD_MOVE', '1', '0', '25', '10', '0']
    def run(self, data: List[str], Z=40, F: int=64):
        gait = data[1]
        x = restriction(int(data[2]), (-35, 35))
        y = restriction(int(data[3]), (-35, 35))
        if gait == "1":
            F = round(map_num(int(data[4]), 2, 10, 126, 22))
        else:
            F = round(map_num(int(data[4]), 2, 10, 171, 45))
        angle = int(data[5])
        z = Z/F
        delay = 0.01
        point = copy.deepcopy(self.body_point)
        # if y < 0:
        #   angle=-angle
        if angle != 0:
            x = 0
        xy = [[0] * 2] * 6
        for i in range(6):
            xy[i][0] = ((point[i][0]*math.cos(angle/180*math.pi) + point[i][1]*math.sin(angle/180*math.pi)-point[i][0])+x)/F
            xy[i][1] = ((-point[i][0]*math.sin(angle/180*math.pi) + point[i][1]*math.cos(angle/180*math.pi)-point[i][1])+y)/F
        if x == 0 and y == 0 and angle == 0:
            self.coordinate_transformation(point)
            self.set_leg_angle()
        elif gait == "1":
            for j in range(F):
                for i in range(3):
                    if j < (F/8):
                        point[2*i][0] = point[2*i][0]-4*xy[2*i][0]
                        point[2*i][1] = point[2*i][1]-4*xy[2*i][1]
                        point[2*i+1][0] = point[2*i+1][0]+8*xy[2*i+1][0]
                        point[2*i+1][1] = point[2*i+1][1]+8*xy[2*i+1][1]
                        point[2*i+1][2] = Z+self.height

                    elif j < (F/4):
                        point[2*i][0] = point[2*i][0]-4*xy[2*i][0]
                        point[2*i][1] = point[2*i][1]-4*xy[2*i][1]
                        point[2*i+1][2] = point[2*i+1][2]-z*8

                    elif j < (3*F/8):
                        point[2*i][2] = point[2*i][2]+z*8
                        point[2*i+1][0] = point[2*i+1][0]-4*xy[2*i+1][0]
                        point[2*i+1][1] = point[2*i+1][1]-4*xy[2*i+1][1]

                    elif j < (5*F/8):
                        point[2*i][0] = point[2*i][0]+8*xy[2*i][0]
                        point[2*i][1] = point[2*i][1]+8*xy[2*i][1]
                        point[2*i+1][0] = point[2*i+1][0]-4*xy[2*i+1][0]
                        point[2*i+1][1] = point[2*i+1][1]-4*xy[2*i+1][1]

                    elif j < (3*F/4):
                        point[2*i][2] = point[2*i][2]-z*8
                        point[2*i+1][0] = point[2*i+1][0]-4*xy[2*i+1][0]
                        point[2*i+1][1] = point[2*i+1][1]-4*xy[2*i+1][1]

                    elif j < (7*F/8):
                        point[2*i][0] = point[2*i][0]-4*xy[2*i][0]
                        point[2*i][1] = point[2*i][1]-4*xy[2*i][1]
                        point[2*i+1][2] = point[2*i+1][2]+z*8

                    elif j < (F):
                        point[2*i][0] = point[2*i][0]-4*xy[2*i][0]
                        point[2*i][1] = point[2*i][1]-4*xy[2*i][1]
                        point[2*i+1][0] = point[2*i+1][0]+8*xy[2*i+1][0]
                        point[2*i+1][1] = point[2*i+1][1]+8*xy[2*i+1][1]

                self.coordinate_transformation(point)
                self.set_leg_angle()
                time.sleep(delay)

        elif gait == "2":
            counter = 0
            number = [5, 2, 1, 0, 3, 4]
            for i in range(6):
                for j in range(int(F/6)):
                    for k in range(6):
                        if number[i] == k:
                            if j < int(F/18):
                                point[k][2] += 18*z
                            elif j < int(F/9):
                                point[k][0] += 30*xy[k][0]
                                point[k][1] += 30*xy[k][1]
                            elif j < int(F/6):
                                point[k][2] -= 18*z
                        else:
                            point[k][0] -= 2*xy[k][0]
                            point[k][1] -= 2*xy[k][1]
                    self.coordinate_transformation(point)
                    self.set_leg_angle()
                    time.sleep(delay)
                    counter += 1


if __name__ == '__main__':
    pass
