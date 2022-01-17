from naoqi import ALProxy
import argparse
import sys
import time
import numpy as np
import cv2
import vision_definitions
import math
import almath
import motion as mot

posture = ALProxy("ALRobotPosture", "192.168.140.245", 9559)
motion = ALProxy("ALMotion", "192.168.140.245", 9559)
tts = ALProxy("ALTextToSpeech", "192.168.140.245", 9559)
video = ALProxy("ALVideoDevice", "192.168.140.245", 9559)

motion.wakeUp()
# motion.angleInterpolationWithSpeed("HeadPitch", [0.5, 0.035], 0.1)

resolution = vision_definitions.kQQVGA
colorSpace = vision_definitions.kBGRColorSpace
fps = 20
nameId = video.subscribe("python_GVM", resolution, colorSpace, fps)
width = 320
height = 240
image = np.zeros((height, width, 3), np.uint8)

mycolor = [0, 38, 14, 254, 0, 255]


def findColor(img):
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # ((142, 192, 56), (200, 255, 255)
    lower = np.array([142, 192, 56])
    upper = np.array([(200, 255, 255)])
    mask = cv2.inRange(imgHSV, lower, upper)
    imgResults = cv2.bitwise_and(img, imgHSV, mask=mask)
    return mask


def getpic():
    while True:
        result = video.getImageRemote(nameId);
        if result == None:
            print 'cannot capture.'
        elif result[6] == None:
            print 'no image data string.'
        else:
            image = (
                np.reshape(np.frombuffer(result[6], dtype='%iuint8' % result[2]), (result[1], result[0], result[2])))
            image = cv2.resize(image, (640, 480))
            imageResult = image.copy()
            cv2.line(imageResult, (320, 0), (320, 480), (10, 0, 200), 1)
            cv2.line(imageResult, (0, 240), (640, 240), (0, 0, 255), 1)
            mask = findColor(image)
            CM, radius = getcontours(mask, imageResult)
            cv2.imshow("M", imageResult)
        return CM, radius


def getcontours(mask, imgResult):
    cont, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    icenter = []
    if len(cont) >= 1:
        maxim = 0
        minE = 1
        for i in range(len(cont)):
            if len(cont[i]) > len(cont[maxim]):
                maxim = i
        contour = cont[maxim]
        cv2.drawContours(imgResult, contour, -1, (255, 0, 0), 3)
        center, radius = cv2.minEnclosingCircle(contour)
        icenter.append(int(center[0]))
        icenter.append(int(center[1]))
        radius = int(radius)
        # print ("Center", icenter, ". Radius", radius)
        if radius > 8 and radius < 200:
            j = icenter[1]
            i = icenter[0]
            cv2.rectangle(imgResult, (i - radius, j - radius), (i + radius, j + radius), (55, 220, 0), 2)
        else:
            i = 0
            j = 0
    else:
        i = 0
        j = 0
        radius = 0
        contour = cont
    CM = [i, j]
    return CM, radius


def tuning_angle(CM):
    to_rad = almath.TO_RAD
    gocquay = (abs(float(CM[0]) - 320.0)) / 640.0 * 67.4 * to_rad
    if CM[0] > 320:
        gocquay = - gocquay
    return gocquay


def pre_process(CM):
    if CM[0] == 0 and CM[1] == 0:
        angle, CM = scan_area(CM)
        motion.angleInterpolationWithSpeed("Head", [0, 0.035], 0.1)
        motion.moveTo(0, 0, angle)
        gocquay = tuning_angle(CM)
        motion.moveTo(0, 0, gocquay)
        CM, radius = getpic()
        return CM
    else:
        gocquay = tuning_angle(CM)
        motion.moveTo(0, 0, gocquay)
        CM, radius = getpic()
        return CM
    pass


def ronate(CM):
    while True:
        CM, radius = getpic()
        cm = pre_process(CM)
        print("Center:", cm)
        if cm[0] >= 315 and cm[0] <= 325:
            return cm


def kickBall():
    isEnabled = True
    motion.wbEnable(isEnabled)
    stateName = "Fixed"
    supportLeg = "Legs"
    motion.wbFootState(stateName, supportLeg)
    isEnable = True
    supportLeg = "Legs"
    motion.wbEnableBalanceConstraint(isEnable, supportLeg)
    supportLeg = "LLeg"
    duration = 1.0
    motion.wbGoToBalance(supportLeg, duration)
    stateName = "Free"
    supportLeg = "RLeg"
    motion.wbFootState(stateName, supportLeg)
    effectorName = "RLeg"
    axisMask = 63
    frame = mot.FRAME_TORSO
    dx = 0.05  # translation axis X (meters)
    dz = 0.02  # translation axis Z (meters)
    dwy = 5.0 * math.pi / 180.0  # rotation axis Y (radian)
    times = [1.0, 1.4, 2.1]
    isAbsolute = False
    targetList = [
        [-0.7 * dx, 0.0, 1.1 * dz, 0.0, +dwy, 0.0],
        [+2.2 * dx, +dx, dz, 0.0, -dwy, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
    motion.positionInterpolation(effectorName, frame, targetList, axisMask, times, isAbsolute)
    isActive = False
    motion.wbEnableEffectorOptimization(effectorName, isActive)
    time.sleep(1.0)
    isEnabled = False
    motion.wbEnable(isEnabled)
    posture.goToPosture("StandInit", 0.5)


def turn_lower():
    video.stopCamera(0)
    video.startCamera(1)
    video.setActiveCamera(1)


def turn_upper():
    video.stopCamera(1)
    video.startCamera(0)
    video.setActiveCamera(0)


def scan_area(CM):
    names = "HeadYaw"
    useSensors = False
    motionAngles = []
    maxAngleScan = 0
    loop = 0
    turn = 0
    turn_upper()
    while maxAngleScan < math.pi / 3:
        maxAngleScan = maxAngleScan + math.pi / 6
        motion.angleInterpolationWithSpeed("Head", [maxAngleScan, 0.035], 0.2)
        commandAngles = motion.getAngles(names, useSensors)
        CM, radius = getpic()
        if CM[0] != 0 and CM[1] != 0:
            return maxAngleScan, CM
        else:
            if maxAngleScan == math.pi / 3:
                loop = loop + 1
                while maxAngleScan > -math.pi / 3:
                    maxAngleScan = maxAngleScan - math.pi / 6
                    motion.angleInterpolationWithSpeed("Head", [maxAngleScan, 0.035], 0.2)
                    commandAngles = motion.getAngles(names, useSensors)
                    CM, radius = getpic()
                    if CM[0] != 0 and CM[1] != 0:
                        return maxAngleScan, CM
                    elif CM[0] == 0 and CM[1] == 0 and loop == 2:
                        motion.moveTo(0, 0, -1)
                        turn = turn + 1
                        print(" Sum of turn", turn)
                        if turn == 4:
                            scan_lower_1()
                        loop = 0


def scan_lower_1():
    turn_lower()
    print("I can't see the ball")
    turn = 0
    while turn <= 7:
        CM, radius = getpic()
        if CM[0] == 0 and CM[1] == 0:
            motion.moveTo(0, 0, -1)
            turn = turn + 1
            print(turn)
        else:
            break
    if CM[0] == 0 and CM[1] == 0:
        turn_upper()
        print(" Upper will be change")


def get_Ready(CM, radius):
    if CM[0] == 0 and CM[1] == 0:
        print(" Lower camera will be used")
        turn_lower()
        CM, radius = getpic()
        if CM[0] == 0 and CM[1] == 0:
            print(" Khong duoc r")
        else:
            center = ronate(CM)
            while center[0] >= 315 and center[0] <= 335:
                motion.moveTo(0.1, 0, 0)
                CM, radius = getpic()
                print('Radius', radius, "cm[1]", CM[1])
                if radius >= 100 and CM[1] > 450:
                    print("I will kick ball.Hihi")
                    kickBall()
                    CM, radius = getpic()
                    if CM[0] == 0 and CM[1] == 0:
                        print("After kick ball, i can't see the ball, i will use cam2")
                        turn_upper()
                        break
                elif CM[0] == 0 and CM[1] == 0:
                    print("I will use upper cam again ")
                    turn_upper()
                    break
                else:
                    cm = pre_process(CM)
        if CM[0] == 0 and CM[1] == 0:
            print("I will d use upper cam")
            turn_upper()


def main():
    while True:
        CM, radius = getpic()
        cm = pre_process(CM)
        if cm[0] > 318 and cm[0] < 322:
            motion.moveTo(0.5, 0, 0)
            CM, radius = getpic()
            get_Ready(CM, radius)
            # exit by [ESC]
        if cv2.waitKey(33) == 27:
            break


if __name__ == "__main__":
    main()
