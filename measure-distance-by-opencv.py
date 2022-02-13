# coding=utf-8
import cv2
import qi
import cv2 as cv
import numpy as np
import almath
import math
import motion
import sys
import vision_definitions
import argparse


def none(x):
    pass


def getImange(session):
    # service motion and posture
    motion_service = session.service('ALMotion')
    posture = session.service('ALRobotPosture')
    motion_service.wakeUp()
    posture.goToPosture('StandInit', 1)
    motion_service.angleInterpolationWithSpeed('Head', [0, 0.30], 0.2)

    # đăng ký sử dụng video
    video = session.service('ALVideoDevice')
    resolution = vision_definitions.kVGA
    colorSpace = vision_definitions.kBGRColorSpace
    fps = 20
    video.setActiveCamera(1)
    topcamera = video.subscribe("python", resolution, colorSpace, fps)
    # camera 1
    focal_length_1 = 806.449169691
    radius_real = 0.05

    winnamed = 'camera'
    cv2.namedWindow(winnamed)
    # (hmin, hmax, smin, smax, vmin, vmax) = (0, 179, 0, 255, 0, 255)
    # cv2.createTrackbar('hmin', winnamed, hmin, 179, none)
    # cv2.createTrackbar('hmax', winnamed, hmax, 179, none)
    # cv2.createTrackbar('smin', winnamed, smin, 255, none)
    # cv2.createTrackbar('smax', winnamed, smax, 255, none)
    # cv2.createTrackbar('vmin', winnamed, vmin, 255, none)
    # cv2.createTrackbar('vmax', winnamed, vmax, 255, none)
    while True:
        # Nhận ảnh từ camera
        result_top = video.getImageRemote(topcamera)
        if result_top == None:
            print 'cannot capture.'
        elif result_top[6] == None:
            print 'no image data string.'
        else:
            # chuyển dữ liệu nhận được sang dạng ảnh
            image_top = (np.reshape(np.frombuffer(result_top[6],
                                                  dtype='%iuint8' % result_top[2]),
                                    (result_top[1], result_top[0], result_top[2])))
            hsv = cv.cvtColor(image_top, cv.COLOR_BGR2HSV)
            # hmin = cv.getTrackbarPos('hmin', winnamed)
            # hmax = cv.getTrackbarPos('hmax', winnamed)
            # smin = cv.getTrackbarPos('smin', winnamed)
            # smax = cv.getTrackbarPos('smax', winnamed)
            # vmin = cv.getTrackbarPos('vmin', winnamed)
            # vmax = cv.getTrackbarPos('vmax', winnamed)
            # # camera bottom (0, 146, 127), (179, 255, 255)
            # # camera top (142, 192, 56), (179, 255, 255)
            # # camera webots (0, 124, 205), (1, 255, 255)
            # mask = cv.inRange(hsv, (hmin, smin, vmin), (hmax, smax, vmax))
            mask = cv.inRange(hsv, (0, 124, 205), (1, 255, 255))
            contours, hier = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                if cv2.contourArea(contour) > 50:
                    # data[4] = 1
                    cnt = contour
                    (x, y), radius = cv.minEnclosingCircle(cnt)
                    print('x: {0}; y: {1}, radius: {2}'.format(int(x), int(y), int(radius)))
                    cx = int(x)  # trọng tâm theo x
                    cy = int(y)  # trọng tâm theo y
                    # Tính khoảng cách
                    bottom = motion_service.getPosition('CameraBottom', motion.FRAME_ROBOT, True)
                    distance = 0
                    print('camera bottom height: {0}'.format(bottom[2]))
                    if video.getActiveCamera() == 1:
                        distance = radius_real * focal_length_1 / int(radius)
                        if distance ** 2 - bottom[2] ** 2 > 0:
                            distance = math.sqrt(distance ** 2 - bottom[2] ** 2)
                        else:
                            distance = -1
                    print('Khoảng cách hiện tại: {0}'.format(distance))
                    cv.circle(image_top, (cx, cy), int(radius), (0, 255, 255), 2)
            cv.imshow(winnamed, image_top)
        # exit by [ESC]
        if cv.waitKey(1) == 27:
            video.releaseImage(topcamera)
            video.unsubscribe(topcamera)
            break


def turnaround():
    pass


if __name__ == '__main__':
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
        print ("Can't connect to Naoqi at ip " + args.ip + " at port " + str(args.port) + "\nPlease check your script "
                                                                                          "arguments. Run with -h option "
                                                                                          "for help.")
        sys.exit(1)
    data = []
    getImange(session)
