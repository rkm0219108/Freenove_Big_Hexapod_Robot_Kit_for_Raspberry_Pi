# coding:utf-8
import math
import os
import time

from mpu6050 import mpu6050

from Kalman import *


class IMU:
    def __init__(self):
        self.Kp = 100
        self.Ki = 0.002
        self.halfT = 0.001

        self.q0 = 1
        self.q1 = 0
        self.q2 = 0
        self.q3 = 0

        self.exInt = 0
        self.eyInt = 0
        self.ezInt = 0
        self.pitch = 0
        self.roll = 0
        self.yaw = 0

        self.sensor = mpu6050(address=0x68, bus=1)
        self.sensor.set_accel_range(mpu6050.ACCEL_RANGE_2G)
        self.sensor.set_gyro_range(mpu6050.GYRO_RANGE_250DEG)

        self.kalman_filter_accel_x = Kalman_filter(0.001, 0.1)
        self.kalman_filter_accel_y = Kalman_filter(0.001, 0.1)
        self.kalman_filter_accel_z = Kalman_filter(0.001, 0.1)

        self.kalman_filter_gyro_x = Kalman_filter(0.001, 0.1)
        self.kalman_filter_gyro_y = Kalman_filter(0.001, 0.1)
        self.kalman_filter_gyro_z = Kalman_filter(0.001, 0.1)

        self.accel_average_data, self.gyro_average_data = self.average_filter()

    def average_filter(self):
        sum_accel_x = 0
        sum_accel_y = 0
        sum_accel_z = 0

        sum_gyro_x = 0
        sum_gyro_y = 0
        sum_gyro_z = 0

        for i in range(100):
            accel_data = self.sensor.get_accel_data()
            gyro_data = self.sensor.get_gyro_data()

            sum_accel_x += accel_data['x']
            sum_accel_y += accel_data['y']
            sum_accel_z += accel_data['z']

            sum_gyro_x += gyro_data['x']
            sum_gyro_y += gyro_data['y']
            sum_gyro_z += gyro_data['z']

        sum_accel_x /= 100
        sum_accel_y /= 100
        sum_accel_z /= 100

        sum_gyro_x /= 100
        sum_gyro_y /= 100
        sum_gyro_z /= 100

        accel_data['x'] = sum_accel_x
        accel_data['y'] = sum_accel_y
        accel_data['z'] = sum_accel_z - 9.8

        gyro_data['x'] = sum_gyro_x
        gyro_data['y'] = sum_gyro_y
        gyro_data['z'] = sum_gyro_z
        return accel_data, gyro_data

    def imu_update(self):
        accel_data = self.sensor.get_accel_data()
        gyro_data = self.sensor.get_gyro_data()

        ax = self.kalman_filter_accel_x.kalman(accel_data['x'] - self.accel_average_data['x'])
        ay = self.kalman_filter_accel_y.kalman(accel_data['y'] - self.accel_average_data['y'])
        az = self.kalman_filter_accel_z.kalman(accel_data['z'] - self.accel_average_data['z'])
        gx = self.kalman_filter_gyro_x.kalman(gyro_data['x'] - self.gyro_average_data['x'])
        gy = self.kalman_filter_gyro_y.kalman(gyro_data['y'] - self.gyro_average_data['y'])
        gz = self.kalman_filter_gyro_z.kalman(gyro_data['z'] - self.gyro_average_data['z'])

        norm = math.sqrt(ax * ax + ay * ay + az * az)

        ax = ax / norm
        ay = ay / norm
        az = az / norm

        vx = 2 * (self.q1 * self.q3 - self.q0 * self.q2)
        vy = 2 * (self.q0 * self.q1 + self.q2 * self.q3)
        vz = self.q0 * self.q0 - self.q1 * self.q1 - self.q2 * self.q2 + self.q3 * self.q3

        ex = (ay * vz - az * vy)
        ey = (az * vx - ax * vz)
        ez = (ax * vy - ay * vx)

        self.exInt += ex * self.Ki
        self.eyInt += ey * self.Ki
        self.ezInt += ez * self.Ki

        gx += self.Kp * ex + self.exInt
        gy += self.Kp * ey + self.eyInt
        gz += self.Kp * ez + self.ezInt

        self.q0 += (-self.q1 * gx - self.q2 * gy - self.q3 * gz) * self.halfT
        self.q1 += (self.q0 * gx + self.q2 * gz - self.q3 * gy) * self.halfT
        self.q2 += (self.q0 * gy - self.q1 * gz + self.q3 * gx) * self.halfT
        self.q3 += (self.q0 * gz + self.q1 * gy - self.q2 * gx) * self.halfT

        norm = math.sqrt(self.q0 * self.q0 + self.q1 * self.q1 + self.q2 * self.q2 + self.q3 * self.q3)
        self.q0 /= norm
        self.q1 /= norm
        self.q2 /= norm
        self.q3 /= norm

        pitch = math.asin(-2*self.q1*self.q3+2*self.q0*self.q2)*57.3
        roll = math.atan2(2*self.q2*self.q3+2*self.q0 * self.q1, -2*self.q1*self.q1-2*self.q2*self.q2+1)*57.3
        yaw = math.atan2(2*(self.q1*self.q2 + self.q0*self.q3), self.q0 * self.q0+self.q1*self.q1-self.q2*self.q2-self.q3*self.q3)*57.3
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw
        return pitch, roll, yaw


# Main program logic follows:
if __name__ == '__main__':
    s = IMU()
    time1 = time.time()
    while True:
        try:
            time.sleep(0.01)
            r, p, y = s.imu_update()
            print(r, p, y)
        except Exception as e:
            print(e)
            os.system("i2cdetect -y 1")
            break
