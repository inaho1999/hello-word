import qi
import math
import motion

session = qi.Session()
session.connect('tcp://127.0.0.1:9559')
motionser = session.service("ALMotion")
posture = session.service('ALRobotPosture')
distance = 0.8
angle = math.pi/3
motionser.wakeUp()
posture.goToPosture('StandInit', 0.5)
postion = motionser.getPosition('Torso', motion.FRAME_WORLD, True)
print(postion)
i = 0
# motionser.moveTo(1.8, 0, 0)
while True:
    x = distance - math.cos(angle) * distance
    y = distance * math.sin(angle)
    motionser.moveTo(x, -y, angle)
    motionser.waitUntilMoveIsFinished()
    i = i + 1
    if i == 13:
        break
print('FINISH')
postion = motionser.getPosition('Torso', motion.FRAME_WORLD, True)
print(postion)
motionser.rest()
