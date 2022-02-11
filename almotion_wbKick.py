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
    dx      = 0.12                 # translation axis X (meters)
    dz      = 0.08                 # translation axis Z (meters)
    dwy     = 10.0*almath.TO_RAD    # rotation axis Y (radian)

    useSensorValues = False

    path = []
    currentTf = []
    try:
        currentTf = motion_service.getTransform(effector, frame, useSensorValues)
    except Exception, errorMsg:
        print str(errorMsg)
        print "This example is not allowed on this robot."
        exit()

    # 1
    targetTf  = almath.Transform(currentTf)
    targetTf *= almath.Transform(-dx+0.1, 0.0, dz)
    targetTf *= almath.Transform().fromRotY(dwy)
    path.append(list(targetTf.toVector()))

    # 2
    targetTf  = almath.Transform(currentTf)
    targetTf *= almath.Transform(dx, 0.0, dz)
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
    # Get the services ALMotion & ALRobotPosture.

    #motion_service = session.service("ALMotion")
    #posture_service = session.service("ALRobotPosture")

    # Wake up robot
    #motion_service.wakeUp()

    # Send robot to Stand Init
    #posture_service.goToPosture("StandInit", 0.5)

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
    #motion_service.wbEnableBalanceConstraint(isEnable, supportLeg)

    # Com go to LLeg
    supportLeg = "LLeg"
    duration   = 2.0
    motion_service.wbGoToBalance(supportLeg, duration)
    motion_service.wbEnableBalanceConstraint(isEnable, supportLeg)
    # RLeg is free
    stateName  = "Free"
    supportLeg = "RLeg"
    motion_service.wbFootState(stateName, supportLeg)

    # RLeg is optimized
    effector = "RLeg"
    axisMask = 63
    frame    = mt.FRAME_WORLD

    # Motion of the RLeg
    times   = [2.0, 2.5, 4.2]
    #tính toán quỹ đạo của chân và đá bóng
    path = computePath(motion_service, effector, frame)
    isActive = False
    motion_service.wbEnableEffectorOptimization(effector, isActive)
    motion_service.transformInterpolations(effector, frame, path, axisMask, times)
    '''
    # Example showing how to Enable Effector Control as an Optimization
    isActive     = False
    motion_service.wbEnableEffectorOptimization(effector, isActive)

    # Com go to LLeg
    supportLeg = "RLeg"
    duration   = 2.0
    motion_service.wbGoToBalance(supportLeg, duration)
    motion_service.wbEnableBalanceConstraint(isEnable, supportLeg)

    # RLeg is free
    stateName  = "Free"
    supportLeg = "LLeg"
    motion_service.wbFootState(stateName, supportLeg)

    effector = "LLeg"
    path = computePath(motion_service, effector, frame)
    motion_service.transformInterpolations(effector, frame, path, axisMask, times)
    '''
    time.sleep(1.0)

    # Deactivate Head tracking
    isEnabled = False
    motion_service.wbEnable(isEnabled)

    # send robot to Pose Init
    posture_service.goToPosture("StandInit", 0.3)

    # Go to rest position
    #motion_service.rest()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9560,
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
    main(motion)
    posture.goToPosture('StandInit', 0.5)
    motion.rest()
