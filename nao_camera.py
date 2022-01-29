import cv2
from naoqi import ALProxy
import numpy as np
import math
import vision_definitions
import almath
import argparse


class cameraNao:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.resolution = vision_definitions.kVGA
        self.colorSpace = vision_definitions.kBGRColorSpace
        self.fps = 20
        self.data = [0, 0, 0, 0, 0, 0, 0, 0]
        try:
            self.video_module = ALProxy('ALVideoDevice', self.ip, self.port)
        except RuntimeError:
            print('fail to connect to video device in tcp://' + str(self.ip) + ':' + str(self.port) + '')
        self.camera = self.video_module.subscribe("python", self.resolution, self.colorSpace, self.fps)
        self.video_module.setActiveCamera(0)
        self.data[5] = 0

    def getImage(self):
        self.image_data = self.video_module.getImageRemote(self.camera)
        if self.image_data == None:
            print('Cannot capture')
            return False
        elif self.image_data[6] == None:
            print('No image data')
            return False
        else:
            self.image = (np.reshape(np.frombuffer(self.image_data[6],
                                                   dtype='%iuint8' % self.image_data[2]),
                                     (self.image_data[1], self.image_data[0], self.image_data[2])
                                     )
                          )
            return True

    def findColor(self):
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 23, 102), (17, 255, 255))
        contours, hier = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            for contour in contours:
                if cv2.contourArea(contour) > 30:
                    (x, y), radius = cv2.minEnclosingCircle(contour)
                    cox = int(x)
                    coy = int(y)
                    self.gocquay = (abs(x - 320.0)) / 640.0 * 67.4 * almath.TO_RAD
                    if self.gocquay > 320:
                        self.gocquay = -self.gocquay
                    cv2.circle(self.image, (cox, coy), int(radius), (0, 200, 100), thickness=2)
                    self.data[2] = radius
                    break
            self.data[0] = self.gocquay
            self.data[3] = 1100


        elif self.data[7] == 1:
            if self.video_module.getActiveCamera() == 0:
                self.video_module.setActiveCamera(1)
                self.data[5] = 1
        else:
            self.data[3] = -1100
            if self.video_module.getActiveCamera() == 1:
                self.video_module.setAciveCamera(0)
                self.data[5] = 0
            if self.data[4] == 6:
                if self.video_module.getActiveCamera() == 0:
                    self.video_module.setActiveCamera(1)
                    self.data[4] += 1
                    self.data[5] = 1
                else:
                    self.video_module.setActiveCamera(0)
                    self.data[5] = 0
                    self.data[4] += 1
            if self.data[4] == 12:
                self.video_module.setAciveCamera(0)
                self.data[5] = 0
                self.data[4] = 0

        return self.image

    def camera(self):
        self.findColor()
        cv2.imshow('cam', self.image)
        return self.data


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")
    args = parser.parse_args()
    naocam = cameraNao(args.ip, args.port)
    while naocam.getImage():
        image = naocam.findColor()
        cv2.imshow('cam', image)
        if cv2.waitKey(1) == 27:
            break
