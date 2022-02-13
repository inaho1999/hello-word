#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: Whole Body Motion - kick"""

import qi
import argparse
import sys
import motion as mt
import time
import almath


def computePath(motion_service, effector, frame):
    dx = 0.05 # translation axis X (meters)
    dz = 0.05  # translation axis Z (meters)
    dwy = 10.0 * almath.TO_RAD  # rotation axis Y (radian)

    useSensorValues = False

    path = []
    currentTf = []
    try:
        currentTf = motion_service.getTransform(effector, frame, useSensorValues)
        print("curent: {0}".format(currentTf))
    except Exception, errorMsg:
        print str(errorMsg)
        print "This example is not allowed on this robot."
        exit()

    # 1
    targetTf = almath.Transform(currentTf)
    targetTf *= almath.Transform(-dx, 0.0, dz)
    targetTf *= almath.Transform().fromRotY(dwy)
    path.append(list(targetTf.toVector()))
    print("target1: {0}".format(targetTf))
    print("target1: {0}".format(targetTf.toVector()))

    # 2
    targetTf = almath.Transform(currentTf)
    targetTf *= almath.Transform(dx + 0.06, 0.0, dz)
    # targetTf *= almath.Transform().fromRotY(dwy)
    path.append(list(targetTf.toVector()))

    # 3
    path.append(currentTf)

    return path


def main(motion_service, posture_service):
    """
    Example of a whole body kick.
    Warning: Needs a PoseInit before executing
             Whole body balancer must be inactivated at the end of the script.
    This example is only compatible with NAO.
    """

    # Activate Whole Body Balancer
    isEnabled  = True
    motion_service.wbEnable(isEnabled)

    # Legs are constrained fixed
    stateName  = "Fixed"
    supportLeg = "Legs"
    motion_service.wbFootState(stateName, supportLeg)

    # Constraint Balance Motion
    isEnable   = True
    supportLeg = "Legs"
    motion_service.wbEnableBalanceConstraint(isEnable, supportLeg)

    # Com go to LLeg
    supportLeg = "LLeg"
    duration   = 2.0
    motion_service.wbGoToBalance(supportLeg, duration)
    # motion_service.wbEnableBalanceConstraint(isEnable, supportLeg)
    # RLeg is free
    stateName  = "Free"
    supportLeg = "RLeg"
    motion_service.wbFootState(stateName, supportLeg)

    # RLeg is optimized
    effector = "RLeg"
    axisMask = 63
    frame    = mt.FRAME_WORLD

    # Motion of the RLeg
    # times   = [2.0, 2.7, 4.2]
    times = [1.0, 1.3, 2.1]
    #tính toán quỹ đạo của chân và đá bóng
    path = computePath(motion_service, effector, frame)
    isActive = True
    # motion_service.wbEnableEffectorOptimization("RLeg", True)

    motion_service.transformInterpolations(effector, frame, path, axisMask, times)
    time.sleep(1.0)

    # Deactivate Head tracking
    isEnabled = False
    motion_service.wbEnable(isEnabled)

    # send robot to Pose Init
    posture_service.goToPosture("StandInit", 0.3)



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
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    motion = session.service('ALMotion')
    posture = session.service('ALRobotPosture')
    motion.wakeUp()
    posture.goToPosture('StandInit', 0.5)
    main(motion, posture)
    posture.goToPosture('StandInit', 0.5)
    motion.rest()
