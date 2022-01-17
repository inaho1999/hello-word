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


def distance_ball(height, radius_pixel, radius_real, focal_length):
    distance = radius_real * focal_length / radius_pixel
    if (distance ** 2 - height ** 2) > 0:
        distance = math.sqrt(distance ** 2 - height ** 2)
    else:
        distance = -1
    return distance


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
    data[5] = 0
    topcamera = video.subscribe("python", resolution, colorSpace, fps)
    data[1] = 0
    data[5] = 0
    # radius of real ball
    radius_real = 0.05
    # camera 1
    focal_length_1 = 903.752521834
    # camera 0
    focal_length_0 = 667.140758904
    # height camera 0
    height_0 = 0.458744257689
    # height camera 1
    height_1 = 0.50361686945

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
                                    (result_top[1],
                                     result_top[0],
                                     result_top[2])
                                    )
                         )
            # hsv image
            hsv = cv.cvtColor(image_top, cv.COLOR_BGR2HSV)
            # binary image
            mask = cv.inRange(hsv, (0, 23, 102), (17, 255, 255))
            # find contour if red ball
            contours, hier = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
            # found red ball
            if len(contours) > 0:
                for contour in contours:
                    if cv2.contourArea(contour) > 50:
                        data[4] = 1
                        cnt = contour
                        (x, y), radius = cv.minEnclosingCircle(cnt)
                        x1, y1, w, h = cv.boundingRect(cnt)
                        # print('x: {0}; y: {1}, radius: {2}'.format(int(x), int(y), int(radius)))
                        cx = int(x)  # trọng tâm theo x
                        cy = int(y)  # trọng tâm theo y
                        cox1 = x - w / 2
                        coy1 = y - h / 2
                        if data[5] == 0:
                            distance = distance_ball(height_0, radius, 0.05, focal_length_0)
                        elif data[5] == 1:
                            distance = distance_ball(height_1, radius, 0.05, focal_length_1)
                        cv.rectangle(image_top, (int(cox1), int(coy1)), (int(cox1 + w), int(coy1 + h)), (0, 0, 255), 2)
                        gocquay = (abs(float(
                            cx) - 320.0)) / 640.0 * 67.4 * almath.TO_RAD  # Tính góc quay cần thiết của robot
                        # dưới dạng radian
                        if cx > 320:
                            gocquay = - gocquay
                        # print('gocquay:_{0}; cx:_{1}; cy:_{2}'.format(gocquay * 180, cx, cy))
                        cv.circle(image_top, (cx, cy), int(radius), (220, 0, 255), 1)
                        data[0] = gocquay
                        data[2] = radius
                        data[3] = 1100  # robot tìm thấy bóng
                        break
            elif data[7] == 1:
                if video.getActiveCamera() == 0:
                    video.setActiveCamera(1)
                    data[5] = 1
                
            else:
                data[3] = -1100  # robot không tìm thấy bóng
                if video.getActiveCamera() == 1:
                    video.setActiveCamera(0)
                    data[5] = 0

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
                    data[1] = 1  # robot lùi lại để tìm bóng

                elif data[4] == 27:
                    video.setActiveCamera(0)
                    data[5] = 0
                    data[4] = 1

            cv.imshow("image_topcamera", image_top)

        # exit by [ESC]
        if cv.waitKey(1) == 27:
            video.releaseImage(topcamera)
            video.unsubscribe(topcamera)
            data[6] = 1
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
            cox = x + w / 2
            coy = y + h / 2
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
        x = radius - radius * math.sin(angle)
        y = radius * math.cos(angle)
        motion_service.moveTo(x, -y, angle)
        motion_service.waitUntilMoveIsFinished()
        count += 1


def rotate_head(session, data):
    # đăng ký sử dụng các module trong robot
    motion_service = session.service('ALMotion')
    posture = session.service('ALRobotPosture')
    motion_service.angleInterpolationWithSpeed('Head', [0, 0.30], 0.2)
    # print('data[0]:', data[0])
    if not motion_service.robotIsWakeUp():
        print('Đánh thức robot')
        motion_service.wakeUp()
        posture.goToPosture('StandInit', 0.5)
    # Robot lùi lại để tìm bóng
    if data[1] == 1:
        print('Bắt đầu lùi lại để tìm bóng')
        motion_service.moveTo(-0.5, 0, 0)
        motion_service.waitUntilMoveIsFinished()
        data[1] = 0
        data[4] += 1
        return 0
    # robot bắt đầu tìm bóng
    elif data[3] == -1100:
        print('bắt đầu tìm bóng')
        motion_service.moveTo(0, 0, math.pi / 3)
        motion_service.waitUntilMoveIsFinished()
        data[4] += 1
        return 0
    # Hiệu chỉnh góc quay khi tìm thấy bóng
    elif abs(data[0]) >= math.pi / 180:
        print('bắt đầu hiệu chỉnh góc quay')
        while abs(data[0]) >= math.pi/180 and data[6] == 0:
            motion_service.moveTo(0, 0, data[0])
        motion_service.moveTo(0, 0, 0)
        # motion_service.moveTo(0, 0, data[0])
        return 0
        # motion.waitUntilMoveIsFinished()
    # robot đi về vị trí bóng
    elif abs(data[0]) < math.pi / 180 and data[3] == 1100:
        print('đi đến vị trí bóng, camera: {0}'.format(int(data[5])))
        # nếu camera hoạt động là camera dưới
        if data[5] == 1:
            # camera 1
            focal_length_1 = 895.697546638
            # radius of real ball
            radius_real = 0.05
            # tìm khoảng cách đến bóng
            bottom = motion_service.getPosition('CameraBottom', motion.FRAME_ROBOT, True)
            distance = distance_ball(bottom[2], data[2], radius_real, focal_length_1)
            print('khoang cach distance: {0}, sử dụng camera: {1}'.format(distance, int(data[5])))
            # # di chuyển đến bóng
            # motion_service.moveTo(distance - 0.2, 0, 0)
            # motion_service.waitUntilMoveIsFinished()
            # time.sleep(1)
            # # tìm khoảng cách sau khi di chuyển
            # bottom = motion_service.getPosition('CameraBottom', motion.FRAME_ROBOT, True)
            # distance = radius_real * focal_length_1 / data[2]
            # distance = math.sqrt(distance ** 2 - bottom[2] ** 2)
            while (distance > 0.1):
                motion_service.moveTo(0.05, 0, 0)
                bottom = motion_service.getPosition('CameraBottom', motion.FRAME_ROBOT, True)
                distance = distance_ball(bottom[2], data[2], radius_real, focal_length_1)
            motion_service.moveTo(0, 0, 0)
            print('khoang cach distance: {0}, sử dụng camera: {1}'.format(distance, int(data[5])))
            # di chuyển xung quanh bóng
            aroung_ball(motion_service, 30, distance*2)
            # dịch chuyển để sút bóng
            motion_service.moveTo(0, 0.05, 0)
            motion_service.waitUntilMoveIsFinished()
            kb.main(motion_service)
            time.sleep(1)
            return 0
        elif data[5] == 0:
            data[7] = 1
            motion_service.moveTo(0.5, 0, 0)
            motion_service.waitUntilMoveIsFinished()
            data[7] = 0
            return 0
            # # camera 0
            # focal_length_0 = 667.140758904
            # top = motion_service.getPosition('CameraTop', motion.FRAME_ROBOT, True)
            # # radius of real ball
            # radius_real = 0.05
            # distance = focal_length_0 * radius_real / data[2]
            # distance = math.sqrt(distance ** 2 - top[2] ** 2)
            # print('khoang cach distance: {0}, sử dụng camera: {1}'.format(distance, int(data[5])))
            # motion_service.moveTo(distance, 0, 0)
            # motion_service.waitUntilMoveIsFinished()
            # time.sleep(1)


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
    data = mlti.Array('d', 8)
    data[4] = 1
    data[6] = 0
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
    p1 = mlti.Process(target=getImange, args=(data,))
    p1.start()
    while True:
        rotate_head(session, data)
        if data[6] == 1:
            break

    p1.join()
