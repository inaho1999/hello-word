# coding=utf-8
from naoqi import ALProxy
import numpy as np
import math
import almath
import motion
import argparse
import almotion_wbKick as kb
import time


class NaoMotion:
    def __init__(self, data, ip='127.0.0.1', port=9559):
        self.ip = ip
        self.port = port
        self.module_motion = ALProxy('ALMotion', self.ip, self.port)
        self.module_posture = ALProxy('ALRobotPosture', self.ip, self.port)
        self.module_motion.wakeUp()
        self.module_posture.goToPosture('StandInit', 0.5)
        self.module_motion.angleInterpolationWithSpeed('Head', [0, 0.30], 0.2)
        self.data = data
        self.focal_length_1 = 895.697546638
        self.radius_real = 0.05

    def aroung_ball(self, angle, radius, count):
        print('Đi xung quanh bóng')
        x = float(radius) - float(radius) * float(math.sin(angle))
        y = float(radius) * math.cos(angle)
        self.module_motion.moveTo(x, -y, angle)
        self.module_motion.waitUntilMoveIsFinished()
        count += 1
        return count

    def findBall(self, data):
        print('bắt đầu tìm bóng')
        self.module_motion.moveTo(0, 0, math.pi / 3)
        self.module_motion.waitUntilMoveIsFinished()
        data[4] += 1
        return data

    def tuneAngle(self, data):
        print('Bát đầu hiệu chỉnh góc quay')
        self.module_motion.moveTo(0, 0, self.data[0])

    def distance_ball(self, radius_pixel):
        print('Tính toán khoảng cách bóng')
        height = self.module_motion.getPosition('CameraBottom', motion.FRAME_ROBOT, True)
        distance = self.radius_real * self.focal_length_1 / radius_pixel
        if (distance ** 2 - height[2] ** 2) > 0:
            distance = math.sqrt(distance ** 2 - height[2] ** 2)
        else:
            distance = -1
        return distance

    def tuneDistance(self, radius_pixel):
        print("Hiệu chỉnh khoảng cách")
        distance = self.distance_ball(radius_pixel)
        walk = distance - 0.25
        self.module_motion.moveTo(walk, 0, 0)
        self.module_motion.waitUntilMoveIsFinished()

    def walkToBall(self, data):
        print('đi đến vị trí bóng, camera: {0}'.format(int(data[5])))
        if data[5] == 1:
            i = 0
            # tìm khoảng cách đến bóng
            bottom = self.module_motion.getPosition('CameraBottom', motion.FRAME_ROBOT, True)
            distance = self.distance_ball(data[2])
            print('khoang cach distance: {0}, sử dụng camera: {1}'.format(distance, int(data[5])))
            while self.distance_ball(data[2]) > 0.25:
                self.module_motion.moveTo(0.05, 0, 0)
                while data[0] > math.pi / 180:
                    self.tuneAngle(data)
            self.module_motion.moveTo(0, 0, 0)
            distance = self.distance_ball(data[2])
            print('khoang cach distance: {0}, sử dụng camera: {1}'.format(distance, int(data[5])))
            # di chuyển xung quanh bóng
            while i < 6:
                i = self.aroung_ball(math.pi/6, distance * 2, i)
                while data[0] > math.pi / 180:
                    self.tuneAngle(data)
                self.tuneDistance(data[2])
                print('Lần thứ {0}'.format(i))
                if i == 6:
                    print('Kết thúc đi quanh bóng')
                    i = 0
                    break
            # dịch chuyển để sút bóng
            self.module_motion.moveTo(0, 0.05, 0)
            self.module_motion.waitUntilMoveIsFinished()
            kb.main(self.module_motion)
            time.sleep(1)
            return 0
        elif data[5] == 0:
            print('Đi đến vị trí bóng, sử dụng camera 0')
            data[7] = 1
            self.module_motion.moveTo(0.5, 0, 0)
            self.module_motion.waitUntilMoveIsFinished()
            data[7] = 0

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")
    args = parser.parse_args()
    data = [0, 0, 87, 1100, 0, 1, 0, 0]
    '''
    data[0]: góc quay hiệu chỉnh
    data[1]: kí hiệu tìm bóng sau khi camera top và bot không tìm thấy
    data[2]: bán kính của bóng
    data[3]: kí hiệu robtot tìm thấy bóng hoặc không tìm thấy bóng
    data[4]: tính robot quay đủ 1 vòng để tìm bóng chưa
    data[5]: xác định camera đang sử dụng
    data[6]: báo hiệu dừng chương trình
    data[7]: robot đang di chuyển về phía bóng
    '''

    walkmotion = NaoMotion(data, args.ip, args.port)
    while True:
        while data[3] != 1100:
            walkmotion.findBall(data)
        while data[0] > math.pi / 180:
            walkmotion.tuneAngle(data)
        while data[3] == 1100:
            walkmotion.walkToBall(data)
        if data[6] == 1:
            break
