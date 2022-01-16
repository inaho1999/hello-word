# coding=utf-8
import multiprocessing as mlti
import os

import qi
import argparse
import sys
import cv2 as cv
import numpy as np
import vision_definitions
import math
import almath
import almotion_wbKick as kb
import time
import motion


def getImange(data):
    session = qi.Session()
    try:
        session.connect("tcp://127.0.0.1:9559")
    except RuntimeError:
        print ("Can't connect to Naoqi at ip " + "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    video = session.service('ALVideoDevice')
    resolution = vision_definitions.kVGA
    colorSpace = vision_definitions.kBGRColorSpace
    fps = 20
    video.setActiveCamera(0)
    topcamera = video.subscribe("python", resolution, colorSpace, fps)
    data[1] = 0
    data[5] = 0

    while True:
        # get image
        result_top = video.getImageRemote(topcamera)
        # if cant capture image
        if result_top == None:
            print 'cannot capture.'
        # if no image data
        elif result_top[6] == None:
            print 'no image data string.'

        else:
            # convert image to form that opencv can read
            image_top = (np.reshape(np.frombuffer(result_top[6],
                                                  dtype='%iuint8' % result_top[2]),
                                    (result_top[1], result_top[0], result_top[2])))
            # hsv image
            hsv = cv.cvtColor(image_top, cv.COLOR_BGR2HSV)
            # binary image
            mask = cv.inRange(hsv, (0, 100, 201), (0, 255, 255))
            # find contour if red ball
            contour, hier = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            # found red ball
            if len(contour) > 0:
                data[4] = 1
                cnt = contour[0]
                (x, y), radius = cv.minEnclosingCircle(cnt)
                x1, y1, w, h = cv.boundingRect(cnt)
                #print('x: {0}; y: {1}, radius: {2}'.format(int(x), int(y), int(radius)))
                cx = int(x)  # trọng tâm theo x
                #print("")
                cy = int(y) # trọng tâm theo y
                cox1 = x - w/2
                coy1 = y - h /2
                # if (coy1 >= 4) and video.getActiveCamera(topcamera) == 0:
                #     video.setActiveCamera(1)
                #     data[5] = 1
                #     #time.sleep(3)
                #
                # if coy1 <= 20 and video.getActiveCamera(topcamera) == 1:
                #     video.setActiveCamera(0)
                #     data[5] = 0
                #     #time.sleep(3)

                cv.rectangle(image_top, (int(cox1), int(coy1)), (int(cox1 + w), int(coy1 + h)), (0, 0, 255), 2)
                gocquay = (abs(float(
                    cx) - 320.0)) / 640.0 * 67.4 * almath.TO_RAD  # Tính góc quay cần thiết của robot
                # dưới dạng radian
                if cx > 320:
                    gocquay = - gocquay
                # print('gocquay:_{0}; cx:_{1}; cy:_{2}'.format(gocquay * 180, cx, cy))

                cv.circle(image_top, (cx, cy), 10, (220, 0, 255), 1)
                data[0] = gocquay
                data[2] = radius
                data[3] = 1100  # robot tìm thấy bóng

            else:
                data[3] = -1100  # robot không tìm thấy bóng

                if data[4] == 9:
                    if video.getActiveCamera(topcamera) == 1:
                        video.setActiveCamera(0)
                        data[4] += 1
                        data[5] = 0
                    else:
                        video.setActiveCamera(1)
                        data[4] += 1
                    data[5] = 1

                elif data[4] == 18:
                    if video.getActiveCamera(topcamera) == 0:
                        video.setActiveCamera(1)
                        data[4] += 1
                        data[5] = 1
                    data[1] = 1 #robot lùi lại để tìm bóng

                elif data[4] == 27:
                    video.setActiveCamera(0)
                    data[5] = 0
                    data[4] = 1

            cv.imshow("image_topcamera", image_top)

        # exit by [ESC]
        if cv.waitKey(1) == 27:
            video.releaseImage(topcamera)
            video.unsubscribe(topcamera)
            break

def aroung_ball(motion_service, angle, radius):
    count = 0
    angle = math.radians(angle)
    while count < 6:
        x = radius - radius*math.sin(angle)
        y = radius*math.cos(angle)
        motion_service.moveTo(x, -y, angle)
        count += 1

def rotate_head(session, data):
    motion_service = session.service('ALMotion')
    posture = session.service('ALRobotPosture')
    # print('data[0]:', data[0])
    if not motion_service.robotIsWakeUp():
        print('Đánh thức robot')
        motion_service.wakeUp()
        posture.goToPosture('StandInit', 0.5)
    if data[1] == 1:
        print('Bắt đầu lùi lại để tìm bóng')
        motion_service.moveTo(-0.5, 0, 0)
        motion_service.waitUntilMoveIsFinished()
        data[1] = 0
        data[4] += 1
    elif data[3] == -1100:
        print('bắt đầu tìm bóng')
        motion_service.moveTo(0, 0, math.pi / 3)
        motion_service.waitUntilMoveIsFinished()
        data[4] += 1
    elif abs(data[0]) >= math.pi / 180:
        print('bắt đầu hiệu chỉnh góc quay')
        motion_service.moveTo(0, 0, data[0])
        #motion.waitUntilMoveIsFinished()
    elif abs(data[0]) <= math.pi / 180 and data[3] == 1100:

        print('đi đến vị trí bóng, camera: {0}'.format(data[5]))

        if data[5] == 1:
            #     print("chuẩn bị sút bóng")
            #     motion.moveTo(0.1, 0, 0)
            #     motion.waitUntilMoveIsFinished()
            #     motion.moveTo(0, 0.05, 0)
            #     motion.waitUntilMoveIsFinished()
            #     print('sút bóng')
            #     kb.main(motion)
            #     motion.waitUntilMoveIsFinished()

            # camera 1
            focal_length_1 = 903.752521834
            # radius of real ball
            radius_real = 0.05
            distance = radius_real * focal_length_1 / data[2]
            bottom = motion_service.getPosition('CameraBottom', motion.FRAME_ROBOT, True)
            distance = math.sqrt(distance ** 2 - bottom[2] ** 2)
            print('khoang cach distance: {0}, sử dụng camera: {1}'.format(distance - 0.1, data[5]))
            motion_service.moveTo(distance, 0, 0)
            motion_service.waitUntilMoveIsFinished()
            time.sleep(1)
            bottom = motion_service.getPosition('CameraBottom', motion.FRAME_ROBOT, True)
            distance = radius_real * focal_length_1 / data[2]
            distance = math.sqrt(distance ** 2 - bottom[2] ** 2)
            aroung_ball(motion_service, 30, distance)
            motion_service.moveTo(0, 0.05, 0)
            motion_service.waitUntilMoveIsFinished()
            kb.main(motion_service)
            motion_service.waitUntilMoveIsFinished()
            # if data[2] > 470:
            #     motion.moveTo(0.1, 0, 0)
            #     motion.waitUntilMoveIsFinished()
            #     motion.moveTo(0, 0.05, 0)
            #     motion.waitUntilMoveIsFinished()
            #     kb.main(motion)
            #     motion.waitUntilMoveIsFinished()
        elif data[5] == 0:
            # if data[2] > 470:
            #     motion.moveTo(0.1, 0, 0)
            #     motion.waitUntilMoveIsFinished()

            # camera 0
            focal_length_0 = 667.140758904
            top = motion_service.getPosition('CameraTop', motion.FRAME_ROBOT, True)
            focal_length_0 = 604.2
            # radius of real ball
            radius_real = 0.05
            distance = focal_length_0*radius_real/data[2]
            distance = math.sqrt(distance ** 2 - top[2] ** 2)
            print('khoang cach distance: {0}, sử dụng camera: {1}'.format(distance, data[5]))
            motion_service.moveTo(distance, 0, 0)
            motion_service.waitUntilMoveIsFinished()



if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) + ".\n"
                                                                                              "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    arr_object = mlti.Array('d', 6)
    arr_object[4] = 1
    '''
    data[0]: góc quay hiệu chỉnh
    data[1]: kí hiệu tìm bóng sau khi camera top và bot không tìm thấy
    data[2]: bán kính của bóng
    data[3]: kí hiệu robtot tìm thấy bóng hoặc không tìm thấy bóng
    data[4]: tính robot quay đủ 1 vòng để tìm bóng chưa
    data[5]: xác định camera đang sử dụng
    '''
    p1 = mlti.Process(target=getImange, args=(arr_object,))
    p1.start()
    while True:
        try:
            rotate_head(session, arr_object)
        except KeyboardInterrupt:
            break

        # print('arr_object[4]: {0}'.format(arr_object[4]) )
        if cv.waitKey(1) == '27':
            break

    p1.join()
