# coding=utf-8
import multiprocessing as mlti
import os

import cv2
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

def findmax(listp):
    max = 0
    for maxi in listp:
        if max < maxi:
            max = maxi
    indexi = np.where(listp == max)
    return max, indexi


def find_person(image):
    classid = []
    with open('coco.names', 'r') as coco:
        print('typeof', type(coco))
        for id in coco:
            classid.append(id.replace('\n', ''))
    coco.close()
    # read network from yolo
    net = cv2.dnn.readNetFromDarknet('yolov4-tiny.cfg', 'yolov4-tiny.weights')
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    # size and confidence threshold
    size = 416
    conf_threshold = 0.9
    nms_threshold = 0.7
    # size of image
    frame_height = image.shape[0]
    frame_width = image.shape[1]
    # find net in image
    blob = cv2.dnn.blobFromImage(image, 1.0 / 255, (size, size), [0, 0, 0], swapRB=False, crop=False)
    net.setInput(blob)
    detections = net.forward()

    bbox = []
    confs = []
    new_classid = []
    cox = None
    coy = None
    for detection in detections:
        if detection[5] > conf_threshold:
            maxi, index = findmax(detection[5:])
            # index_of_coco = index[0][0]
            # print(f'detection: {detection}')
            # print(f'max:{max}, index:{index[0][0]}, name:{classid[index_of_coco]} ')
            toado = detection[0:4]
            w, h = int(detection[2] * frame_width), int(detection[3] * frame_height)
            x, y = int(detection[0] * frame_width - w / 2), int(detection[1] * frame_height - h / 2)
            cox = x + w/2
            coy = y + h/2
            print('toado: {0}, {1}, {2}, {3}'.format(x, y, w, h))
            bbox.append([x, y, w, h])
            confs.append(float(maxi))
            new_classid.append('person')
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0))
            label = 'person'
            label = label + ': ' + ('%.4f' % maxi)
            label_size, base_line = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, fontScale=2, thickness=3)
            cv2.rectangle(image, (x, y - label_size[1]),
                          (x + label_size[0], y + base_line),
                          (255, 255, 255), cv2.FILLED)
            cv2.putText(image, label.upper(), (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, fontScale=2, color=(0, 0, 255), thickness=3)
            break
    # indices = cv2.dnn.NMSBoxes(bbox, confs, conf_threshold, nms_threshold)
    # print('indices: {0}'.format(indices))
    # if len(indices) > 0:
    #     for i in indices:
    #         box = bbox[i]
    #         x, y, w, h = box[0], box[1], box[2], box[3]
    #         cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0))
    #         label = 'person'
    #         label = label + ': ' + ('%.4f' % confs[i])
    #         label_size, base_line = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, fontScale=2, thickness=3)
    #         cv2.rectangle(image, (x, y - label_size[1]),
    #                       (x + label_size[0], y + base_line),
    #                       (255, 255, 255), cv2.FILLED)
    #         cv2.putText(image, label.upper(), (x, y),
    #                     cv2.FONT_HERSHEY_SIMPLEX, fontScale=2, color=(0, 0, 255), thickness=3)
    return cox, coy

def aroung_ball(motion_service, angle, radius):
    count = 0
    angle = math.radians(angle)
    while count < 6:
        x = radius - radius*math.sin(angle)
        y = radius*math.cos(angle)
        motion_service.moveTo(x, -y, angle)
        count += 1
        time.sleep(1)

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
            # camera 1
            focal_length_1 = 903.752521834
            # radius of real ball
            radius_real = 0.05
            # tìm khoảng cách đến bóng
            distance = radius_real * focal_length_1 / data[2]
            bottom = motion_service.getPosition('CameraBottom', motion.FRAME_ROBOT, True)
            distance = math.sqrt(distance ** 2 - bottom[2] ** 2)
            print('khoang cach distance: {0}, sử dụng camera: {1}'.format(distance - 0.1, data[5]))
            # di chuyển đến bóng
            motion_service.moveTo(distance, 0, 0)
            motion_service.waitUntilMoveIsFinished()
            time.sleep(1)
            # tìm khoảng cách sau khi di chuyển
            bottom = motion_service.getPosition('CameraBottom', motion.FRAME_ROBOT, True)
            distance = radius_real * focal_length_1 / data[2]
            distance = math.sqrt(distance ** 2 - bottom[2] ** 2)
            # di chuyển xung quanh bóng
            aroung_ball(motion_service, 30, distance)
            # dịch chuyển để sút bóng
            motion_service.moveTo(0, 0.05, 0)
            motion_service.waitUntilMoveIsFinished()
            kb.main(motion_service)

        elif data[5] == 0:
            focal_length_0 = 667.140758904
            top = motion_service.getPosition('CameraTop', motion.FRAME_ROBOT, True)
            # radius of real ball
            radius_real = 0.05
            distance = focal_length_0*radius_real/data[2]
            distance = math.sqrt(distance ** 2 - top[2] ** 2)
            print('khoang cach distance: {0}, sử dụng camera: {1}'.format(distance, int(data[5])))
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
    arr_object = mlti.Array('d', 8)
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
