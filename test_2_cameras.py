# coding=utf-8
import time

from naoqi import ALProxy
import cv2
import numpy as np
import math
import vision_definitions
import almath
import argparse
import multiprocessing as mlti
import motion
import almotion_wbKick as kb


def getImages(ip, port, data):
    #
    videos = ALProxy('ALVideoDevice', ip, port)
    Name = 'test2camera'
    CameraIndexs = (0, 1)
    Resolutions = (vision_definitions.kVGA, vision_definitions.kVGA)
    ColorSpaces = (vision_definitions.kBGRColorSpace, vision_definitions.kBGRColorSpace)
    FPS = 24
    twocameras = videos.subscribeCameras(Name, CameraIndexs, Resolutions, ColorSpaces, FPS)
    while True:
        images = videos.getImagesRemote(twocameras)
        # print('images shape: 0'.format(np.shape(images)))
        if images == None:
            print('Không thể chụp ảnh')
        elif images[0][6] == None or images[1][6] == None:
            print('Không có ảnh')
        else:
            image0 = np.reshape(np.frombuffer(images[0][6], dtype='%iuint8' % images[0][2]),
                                (images[0][1], images[0][0], images[0][2]))
            image1 = np.reshape(np.frombuffer(images[1][6], dtype='%iuint8' % images[1][2]),
                                (images[1][1], images[1][0], images[1][2]))
            hsv0 = cv2.cvtColor(image0, cv2.COLOR_BGR2HSV)
            hsv1 = cv2.cvtColor(image1, cv2.COLOR_BGR2HSV)
            mask0 = cv2.inRange(hsv0, (0, 129, 204), (0, 255, 255))
            mask1 = cv2.inRange(hsv1, (0, 129, 204), (0, 255, 255))
            contours0, hier0 = cv2.findContours(mask0, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours1, hier1 = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            found_ball = False
            if len(contours0) > 0:
                data[5] = 0
                for contour in contours0:
                    if cv2.contourArea(contour) > 100:
                        (x, y), radius = cv2.minEnclosingCircle(contour)
                        cox = int(x)
                        coy = int(y)
                        cv2.circle(image0, (cox, coy), int(radius), (220, 0, 255), 3)
                        percent = 0
                        for toado in contour:
                            xcnt = toado[0][0]
                            ycnt = toado[0][1]
                            rho = math.sqrt(pow(x-xcnt, 2)+pow(y-ycnt, 2))
                            if percent < abs(rho - radius)/radius:
                                percent = abs(rho - radius)/radius
                        if percent > 0.3:
                            continue
                        # Tính góc quay cần thiết của robot dưới dạng radian
                        gocquay = (abs(float(cox) - 320.0)) / 640.0 * 60.9 * almath.TO_RAD
                        if cox > 320:
                            gocquay = -gocquay
                        data[0] = gocquay
                        data[2] = radius
                        data[3] = 1100
                        found_ball = True
                        break
                        # print(x, y)
            elif len(contours1) > 0 and found_ball is False:
                data[5] = 1
                for contour in contours1:
                    if cv2.contourArea(contour) > 100:
                        (x, y), radius = cv2.minEnclosingCircle(contour)
                        cox = int(x)
                        coy = int(y)
                        percent = 0
                        for toado in contour:
                            xcnt = toado[0][0]
                            ycnt = toado[0][1]
                            rho = math.sqrt(pow(x - xcnt, 2) + pow(y - ycnt, 2))
                            if percent < abs(rho - radius) / radius:
                                percent = abs(rho - radius) / radius
                        if percent > 0.3:
                            continue
                        cv2.circle(image1, (cox, coy), int(radius), (220, 0, 255), 3)
                        # Tính góc quay cần thiết của robot dưới dạng radian
                        gocquay = (abs(float(cox) - 320.0)) / 640.0 * 60.9 * almath.TO_RAD
                        if cox > 320:
                            gocquay = -gocquay
                        data[0] = gocquay
                        data[2] = radius
                        data[3] = 1100
                        break
                        # print(x, y)
            else:
                print('Không tìm thấy bóng')
                data[5] = 2
                data[3] = -1100
            cv2.imshow('camera0', image0)
            cv2.imshow('camera1', image1)
        if cv2.waitKey(1) == 27:
            data[6] = 1
            break
    cv2.destroyAllWindows()
    videos.releaseImages(twocameras)
    videos.unsubscribe(twocameras)

def distance_ball(height, radius_pixel, radius_real, focal_length):
    distance = radius_real * focal_length / radius_pixel
    if (distance ** 2 - height ** 2) > 0:
        distance = math.sqrt(distance ** 2 - height ** 2)
    else:
        distance = -1
    return distance


def aroung_ball(motion_service, angle, radius):
    x = radius - radius * math.sin(angle)
    y = radius * math.cos(angle)
    motion_service.moveTo(x, (-y), angle)
    motion_service.waitUntilMoveIsFinished()


def declareALproxy(ip, port, data):
    ALmotion = ALProxy('ALMotion', ip, port)
    ALPosture = ALProxy('ALRobotPosture', ip, port)
    while data[6] == 0:
        ControlNao(ALmotion, ALPosture)


def ControlNao(ALmotion, ALPosture):
    # Đánh thức robot
    if not ALmotion.robotIsWakeUp():
        print('Đánh thức robot')
        ALmotion.wakeUp()
        ALPosture.goToPosture('StandInit', 0.5)
    # Robot vào tư thế chuẩn bị di chuyển
    ALmotion.moveInit()
    ALmotion.setMoveArmsEnabled(True, True)
    ALmotion.angleInterpolationWithSpeed('Head', [0, 0.30], 0.2)
    # Robot quay góc pi/3 để tìm bóng
    time.sleep(1)
    if data[3] == -1100:
        print('Bắt đầu tìm bóng')
        ALmotion.moveTo(0, 0, math.pi / 3)
        ALmotion.waitUntilMoveIsFinished()
        return 0
    # Robot chỉnh bóng về giữa khung hình
    elif data[3] == 1100 and abs(data[0]) >= math.pi / 180:
        print('Hiệu chỉnh góc quay của robot')
        while abs(data[0]) >= math.pi / 180 and data[6] == 0:
            ALmotion.moveTo(0, 0, data[0])
        ALmotion.moveTo(0, 0, 0)
        return 0
    # Robot tiếp cận và đá bóng
    elif data[3] == 1100 and abs(data[0]) <= math.pi / 180:
        print('Đi đến vị trí bóng, camera {0}:'.format(int(data[5])))
        if data[5] == 1:
            # camera 1 focal length
            focal_length_1 = 844.014966275
            # Bán kính thực của bóng đơn vị SI
            radius_real = 0.05
            # tìm khoảng cách đến bóng
            bottom = ALmotion.getPosition('CameraBottom', motion.FRAME_ROBOT, True)     # Chiều cao của camera dưới
            distance = distance_ball(bottom[2], data[2], radius_real, focal_length_1)   # Tính khoảng cách của robot đến bóng
            print('khoang cach distance: {0}, sử dụng camera: {1}'.format(distance, int(data[5])))
            # điều chỉnh khoảng cách bóng và robot ít hơn 0.25m
            while distance > 0.25:
                ALmotion.moveTo(0.02, 0, 0)
                bottom = ALmotion.getPosition('CameraBottom', motion.FRAME_ROBOT, True)
                distance = distance_ball(bottom[2], data[2], radius_real, focal_length_1)
                print('khoang cach distance: {0}, sử dụng camera: {1}'.format(distance, int(data[5])))
            # điều chỉnh khoảng cách bóng và robot lớn hơn 0.2m
            while distance < 0.2:
                ALmotion.moveTo(-0.02, 0, 0)
                bottom = ALmotion.getPosition('CameraBottom', motion.FRAME_ROBOT, True)
                distance = distance_ball(bottom[2], data[2], radius_real, focal_length_1)
                print('khoang cach distance: {0}, sử dụng camera: {1}'.format(distance, int(data[5])))
            if ALmotion.stopMove():
                print('Đã dừng')
            print('khoang cach distance: {0}, sử dụng camera: {1}'.format(distance, int(data[5])))
            # image = ALCam.getImageRemote(topcamera)
            if data[0] > math.pi/180:
                return 0
            if True:
                print("Đá bóng")
                ALmotion.moveTo(0.2, 0.15, 0)
                ALmotion.waitUntilMoveIsFinished()
                kb.main(ALmotion, ALPosture)
                return 0
            # di chuyển xung quanh bóng
            # aroung_ball(ALmotion, math.pi / 6, distance)
            # return 0
        # Nếu bóng ở camera trên thì tiếp cận từng đoạn 0.5m
        elif data[5] == 0:
            ALmotion.moveTo(0.5, 0, 0)
            ALmotion.waitUntilMoveIsFinished()
            return 0


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")
    args = parser.parse_args()
    ip = args.ip
    port = args.port
    data = mlti.Array('d', 8)
    '''
    data[0]: góc quay hiệu chỉnh
    data[1]: tìm người
    data[2]: bán kính của bóng
    data[3]: kí hiệu robtot tìm thấy bóng hoặc không tìm thấy bóng
    data[4]: tính robot quay đủ 1 vòng để tìm bóng chưa
    data[5]: xác định camera đang sử dụng
    data[6]: báo hiệu dừng chương trình
    data[7]: robot đang di chuyển về phía bóng
    '''
    data[4] = 0
    data[6] = 0
    data[1] = 0
    thread_image = mlti.Process(target=getImages, args=(ip, port, data))
    thread_image.start()
    declareALproxy(args.ip, args.port, data)
    thread_image.join()
    print('Kết thúc chương trình')
