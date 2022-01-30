#!/usr/bin/python
# coding=utf-8

#                          License Agreement
#                         3-clause BSD License
#
#       Copyright (C) 2018, Xperience.AI, all rights reserved.
#
# Third party copyrights are property of their respective owners.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
#
#   * Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#   * Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
#   * Neither the names of the copyright holders nor the names of the contributors
#     may be used to endorse or promote products derived from this software
#     without specific prior written permission.
#
# This software is provided by the copyright holders and contributors "as is" and
# any express or implied warranties, including, but not limited to, the implied
# warranties of merchantability and fitness for a particular purpose are disclaimed.
# In no event shall copyright holders or contributors be liable for any direct,
# indirect, incidental, special, exemplary, or consequential damages
# (including, but not limited to, procurement of substitute goods or services;
# loss of use, data, or profits; or business interruption) however caused
# and on any theory of liability, whether in contract, strict liability,
# or tort (including negligence or otherwise) arising in any way out of
# the use of this software, even if advised of the possibility of such damage.
# coding=utf-8
from __future__ import print_function

import cv2
import qi
import numpy as np
import almath
import math
import motion
import sys
import vision_definitions
import argparse


def none(x):
    pass


def findmax(listp):
    max = 0
    for maxi in listp:
        if max < maxi:
            max = maxi
    indexi = np.where(listp == max)
    return max, indexi


def getImange(ip, port):
    # source = cv2.VideoCapture('test.webm')
    # source = cv2.VideoCapture('race_car_out.avi')
    # source = cv2.VideoCapture('video1.avi')
    # source = cv2.VideoCapture('output_human.avi')
    source = cv2.VideoCapture(0)
    width = int(source.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(source.get(cv2.CAP_PROP_FRAME_HEIGHT))
    frame_size = (width, height)
    fps = 24
    mjpg = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
    writter = cv2.VideoWriter('detect_human_with_yolo.avi', mjpg, fps, frame_size)
    classid = []
    with open('coco.names', 'r') as coco:
        print('typeof', type(coco))
        for id in coco:
            classid.append(id.replace('\n', ''))
    coco.close()

    # print(f'classid:{classid}')
    # motionse = session.service('ALMotion')
    # đọc file pkg và weight của yolo
    # net = cv2.dnn.readNetFromDarknet("yolov4-tiny.cfg", "yolov4-tiny.weights")
    net = cv2.dnn.readNetFromDarknet('yolov4.cfg', 'yolov4.weights')
    # thiết lập back_end và target
    net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
    net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
    # Model parameters
    in_width = 608
    in_height = 608
    mean = [104, 117, 123]
    conf_threshold = 0.7
    nms_threshold = 0.3
    # motionse.wakeUp()
    # service video device
    # video = session.service('ALVideoDevice')
    # resolution = vision_definitions.kVGA
    # colorSpace = vision_definitions.kBGRColorSpace
    # fps = 20
    # video.setActiveCamera(0)
    # topcamera = video.subscribe("python", resolution, colorSpace, fps)
    winnamed = 'camera'
    cv2.namedWindow(winnamed)
    # (hmin, hmax, smin, smax, vmin, vmax) = (0, 255, 0, 255, 0, 255)
    # cv2.createTrackbar('hmin', winnamed, 0, 255, none)
    # cv2.createTrackbar('hmax', winnamed, 0, 255, none)
    # cv2.createTrackbar('smin', winnamed, 0, 255, none)
    # cv2.createTrackbar('smax', winnamed, 0, 255, none)
    # cv2.createTrackbar('vmin', winnamed, 0, 255, none)
    # cv2.createTrackbar('vmax', winnamed, 0, 255, none)
    while True:
        # get image
        # result_top = video.getImageRemote(topcamera)
        # if result_top == None:
        #     print 'cannot capture.'
        # elif result_top[6] == None:
        if False:
            print('no image data string.')
        else:
            has_frame, frame = source.read()
            if not has_frame:
                break
            # frame = (np.reshape(np.frombuffer(result_top[6],
            #                                   dtype='%iuint8' % result_top[2]),
            #                     (result_top[1], result_top[0], result_top[2])))

            # frame = cv2.flip(frame, 1)  # xoay ảnh theo đúng chiều
            frame_height = frame.shape[0]
            frame_width = frame.shape[1]

            # Create a 4D blob from a frame.
            blob = cv2.dnn.blobFromImage(frame, 1.0 / 255, (in_width, in_height), [0, 0, 0], swapRB=False, crop=False)
            # Run a model
            net.setInput(blob)
            detections = net.forward()
            # print(f'detections: {detections.shape}')
            # print(f'detections.shape[0]: {detections[0]}')
            print(detections.shape)
            # print(detections[0])
            bbox = []
            confs = []
            new_classid = []
            # Tìm đối tượng xuất hiện trong ảnh
            for detection in detections:
                if detection[5] > 0.7:
                    print('Tìm thấy 1 người')
                    print(detection)
                    maxi, index = findmax(detection[5:])
                    index_of_coco = index[0][0]
                    # index_of_coco = 0
                    # print('detection: {0}'.format(detection))
                    # print('maxima:{0}, index:{1}, name:{2} '.format(max, index[0][0], classid[index_of_coco]))
                    toado = detection[0:4]
                    w, h = int(detection[2] * frame_width), int(detection[3] * frame_height)
                    x, y = int(detection[0] * frame_width - w / 2), int(detection[1] * frame_height - h / 2)
                    # print(f'toado: {x, y, w, h}')
                    bbox.append([x, y, w, h])
                    confs.append(float(maxi))
                    new_classid.append(classid[index_of_coco])
                    # cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0))
                    # label = classid[index_of_coco]
                    # #label = label + ': ' + ('%.4f' % confs[i])
                    # label_size, base_line = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, fontScale=2, thickness=3)
                    # cv2.rectangle(frame, (x, y - label_size[1]),
                    #               (x + label_size[0], y + base_line),
                    #               (255, 255, 255), cv2.FILLED)
                    # cv2.putText(frame, label.upper(), (x, y),
                    #             cv2.FONT_HERSHEY_SIMPLEX, fontScale=2, color=(0, 0, 255), thickness=3)

            indices = cv2.dnn.NMSBoxes(bbox, confs, conf_threshold, nms_threshold)
            print('indices: {0}'.format(indices))
            if len(indices) > 0:
                for i in indices:
                    box = bbox[i]
                    x, y, w, h = box[0], box[1], box[2], box[3]
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0))
                    label = classid[index_of_coco]
                    label = label + ': ' + ('%.4f' % confs[i])
                    label_size, base_line = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, fontScale=2, thickness=3)
                    cv2.rectangle(frame, (x, y - label_size[1]),
                                  (x + label_size[0], y + base_line),
                                  (255, 255, 255), cv2.FILLED)
                    cv2.putText(frame, label.upper(), (x, y),
                                cv2.FONT_HERSHEY_SIMPLEX, fontScale=2, color=(0, 0, 255), thickness=3)
            t, _ = net.getPerfProfile()
            label = 'Inference time: %.2f ms' % (t * 1000.0 / cv2.getTickFrequency())
            cv2.putText(frame, label, (0, 15), cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 255, 0), thickness=2)
            writter.write(frame)
            cv2.imshow(winnamed, frame)

        # exit by [ESC]
        if cv2.waitKey(1) == 27:
            cv2.cuda.printShortCudaDeviceInfo(cv2.cuda.getDevice())
            source.release()
            writter.release()
            # video.releaseImage(topcamera)
            # video.unsubscribe(topcamera)
            # motionse.rest()
            break


def turnaround():
    pass


def detectionHuman():
    from imutils.object_detection import non_max_suppression
    # from imutils import paths
    import numpy as np
    # import argparse
    import imutils
    import cv2
    # construct the argument parse and parse the arguments

    # cap = cv2.VideoCapture(0)
    """
    cap = cv2.VideoCapture(0)
    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        # Our operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Display the resulting frame
        cv2.imshow('frame',gray)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
    """
    """
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--images", required=True, help="path of image file")
    args = vars(ap.parse_args())
    """
    # initialize the HOG descriptor/person detector
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    cap = cv2.VideoCapture(0)
    # loop over the image paths
    while (True):
        # capture frame by frame
        ret, image = cap.read()
        image = imutils.resize(image, width=min(400, image.shape[1]))
        orig = image.copy()

        # detect people in the image
        (rects, weights) = hog.detectMultiScale(image, winStride=(4, 4),
                                                padding=(8, 8), scale=1.05)

        # draw the original bounding boxes
        for (x, y, w, h) in rects:
            cv2.rectangle(orig, (x, y), (x + w, y + h), (0, 0, 255), 2)
            print(x, y)
        # apply non-maxima suppression to the bounding boxes using a
        # fairly large overlap threshold to try to maintain overlapping
        # boxes that are still people
        rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
        pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)

        # draw the final bounding boxes
        for (xA, yA, xB, yB) in pick:
            cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)

        # show some information on the number of bounding boxes
        # filename = imagePath[imagePath.rfind("/") + 1:]
        print("[INFO]  {} original boxes, {} after suppression".format(
            len(rects), len(pick)))

        # show the output images
        cv2.imshow("Before NMS", orig)
        cv2.imshow("After NMS", image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="192.168.161.245",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")
    session = 1
    args = parser.parse_args()
    # session = qi.Session() try: session.connect("tcp://" + args.ip + ":" + str(args.port)) except RuntimeError:
    # print ("Can't connect to Naoqi at ip " + args.ip + " at port " + str(args.port) + "\nPlease check your script "
    # "arguments. Run with -h option " "for help.") sys.exit(1)
    getImange(args.ip, args.port)
