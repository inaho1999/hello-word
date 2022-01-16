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
from pynput import keyboard

def none(x):
    pass

def getImange(session):
    # session = qi.Session()
    # try:
    #     session.connect("tcp://127.0.0.1:9559")
    # except RuntimeError:
    #     print ("Can't connect to Naoqi at ip " + "Please check your script arguments. Run with -h option for help.")
    #     sys.exit(1)
    # traker reall ball
    redball_detection = session.service('ALRedBallDetection')
    tracker = session.service('ALTracker')
    targetName = 'RedBall'
    diameterOfBall = 0.05
    tracker.registerTarget(targetName, diameterOfBall)
    mode = 'Move'
    tracker.setMode(mode)
    tracker.toggleSearch(False)

    # service motion and posture
    motion_service = session.service('ALMotion')
    posture = session.service('ALRobotPosture')
    motion_service.wakeUp()
    posture.goToPosture('StandInit', 1)
    # Find distance of cameras
    bottom = motion_service.getPosition('CameraBottom', motion.FRAME_ROBOT, True)
    top = motion_service.getPosition('CameraTop', motion.FRAME_ROBOT, True)
    print('Camera 0 position {0}'.format(top[2]))
    print('Camera 1 position {0}'.format(bottom[2]))


    # service video device
    video = session.service('ALVideoDevice')
    resolution = vision_definitions.kVGA
    colorSpace = vision_definitions.kBGRColorSpace
    fps = 20
    video.setActiveCamera(0)
    topcamera = video.subscribe("python", resolution, colorSpace, fps)
    # data[1] = 0
    # data[5] = 0
    #camera 0
    focal_length = 672.975482466
    # # camera 1
    # focal_length = 951.407904108
    winnamed = 'camera'
    cv2.namedWindow(winnamed)
    # (hmin, hmax, smin, smax, vmin, vmax) = (0, 255, 0, 255, 0, 255)
    # cv2.createTrackbar('hmin', winnamed, 0, 255, none)
    # cv2.createTrackbar('hmax', winnamed, 0, 255, none)
    # cv2.createTrackbar('smin', winnamed, 0, 255, none)
    # cv2.createTrackbar('smax', winnamed, 0, 255, none)
    # cv2.createTrackbar('vmin', winnamed, 0, 255, none)
    # cv2.createTrackbar('vmax', winnamed, 0, 255, none)
    tracker.track(targetName)
    tracker.setRelativePosition([-0.15, 0.15, 0, 0.1, 0.1, 0.1])
    print(tracker.getRelativePosition())
    while True:
        # get image
        result_top = video.getImageRemote(topcamera)
        if result_top == None:
            print 'cannot capture.'
        elif result_top[6] == None:
            print 'no image data string.'
        else:
            image_top = (np.reshape(np.frombuffer(result_top[6],
                                                  dtype='%iuint8' % result_top[2]),
                                    (result_top[1], result_top[0], result_top[2])))
            # if tracker.isTargetLost():
            #     print('Doi camera')
            #     if video.getActiveCamera() == 0:
            #         video.setActiveCamera(1)
            #     else:
            #         video.setActiveCamera(0)
            hsv = cv.cvtColor(image_top, cv.COLOR_BGR2HSV)
            # hmin = cv.getTrackbarPos('hmin', winnamed)
            # hmax = cv.getTrackbarPos('hmax', winnamed)
            # smin = cv.getTrackbarPos('smin', winnamed)
            # smax = cv.getTrackbarPos('smax', winnamed)
            # vmin = cv.getTrackbarPos('vmin', winnamed)
            # vmax = cv.getTrackbarPos('vmax', winnamed)
            mask = cv.inRange(hsv, (0, 44, 79), (66, 255, 255))
            contour, hier = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            if len(contour) > 0:
                # data[4] = 1
                cnt = contour[0]
                # (x, y), radius = cv.minEnclosingCircle(cnt)
                # print('x: {0}; y: {1}, radius: {2}'.format(int(x), int(y), int(radius)))
                # cx = int(x)  # trọng tâm theo x
                # # print("")
                # cy = int(y)  # trọng tâm theo y
                # if cy > 470 and video.getActiveCamera(topcamera) == 0:
                #     video.setActiveCamera(1)
                #     # data[5] = 1
                # if cy < 10 and video.getActiveCamera(topcamera) == 1:
                #     video.setActiveCamera(0)
                #     # data[5] = 0

                # distance = 0.05*focal_length/radius
                # print('Khoảng cách đến camera: {0}'.format(distance))
                # distance = math.sqrt(distance**2-0.55*2)
                # print('Khoảng cách hiện tại: {0}'.format(distance))
                # cv.circle(image_top, (cx, cy), int(radius), (0, 0, 255), 2)
                # gocquay = (abs(float(
                #     cx) - 320.0)) / 640.0 * 67.4 * almath.TO_RAD  # Tính góc quay cần thiết của robot
            cv.imshow(winnamed, image_top)
            if tracker.isActive():
                print('Tracker đang hoạt động')

        # exit by [ESC]
        if cv.waitKey(1) == 27:
            print('Tracker target: {0}'.format(tracker.getActiveTarget()))
            print('Tracker effector: {0}'.format(tracker.getEffector()))
            print('Tracker maximum distance detection {0}'.format(tracker.getMaximumDistanceDetection()))
            print('Tracker mode: {0}'.format(tracker.getMode()))
            print('Tracker target: {0}'.format(tracker.getRegisteredTargets()))
            print('Tracker target position: {0}'.format(tracker.getTargetPosition(motion.FRAME_ROBOT)))
            print('Tracker search: {0}'.format(tracker.isSearchEnabled()))
            #print('Sensor name: {0}'.format(motion_service.getSensorNames()))
            video.releaseImage(topcamera)
            video.unsubscribe(topcamera)
            tracker.stopTracker()
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
    while True:
        getImange(session)
        break
