#!/usr/bin/env pybricks-micropython

"""
Example LEGO® MINDSTORMS® EV3 Robot Educator Driving Base Program
-----------------------------------------------------------------

This program requires LEGO® EV3 MicroPython v2.0.
Download: https://education.lego.com/en-us/support/mindstorms-ev3/python-for-ev3

Building instructions can be found at:
https://education.lego.com/en-us/support/mindstorms-ev3/building-instructions#robot
"""

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from LineFollowBeh import LinefollowBeh,PIDParam
from RampBeh import RampBeh,RampParam
from gripper import operate_gripper
from SensorReader import SensorReading
from CanRelatedBeh import CanRelatedBeh

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the motors
left_motor = Motor(Port.D)
right_motor = Motor(Port.A)

#Initialize sensors
SensorReader=SensorReading()

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

threshold=13
k=10
ki=0
kd=0
baseSpeed=50
speedUpFactor=2
speedDownFactor=0.5

pidParam=PIDParam(k,ki,kd)
rampParams=RampParam(speedUpFactor,speedDownFactor)


LineFollower=LinefollowBeh(pidParam,threshold)
RampAdjuster=RampBeh(rampParams,baseSpeed)
CanRelatedBehs=CanRelatedBeh(robot,SensorReader,baseSpeed)
speed=baseSpeed


# once=False
# def GrabCan():
#     robot.stop()
#     robot.turn(-10)
    
#     global once
#     if once==True:
#         return
#     distance=UltraSensor.distance()
#     ev3.speaker.beep(500,200)
#     while(distance>40):
#         robot.drive(speed,0)
#         distance=UltraSensor.distance()
#     robot.stop()
#     robot.straight(20)
#     ev3.speaker.beep(500,100)
#     operate_gripper()
#     robot.turn(230)
#     robot.stop()
#     once=True

prevTurning=0

while True:
    SensorReader.GetInput()

    if SensorReader.endOfLine and not SensorReader.canGrabbed:
        ev3.speaker.beep(SensorReader.distance,100)
        CanRelatedBehs.SearchCan()


    # if SensorReader.potentialCan and not SensorReader.canGrabbed:
    #     ev3.speaker.beep(700,100)
    #     CanRelatedBehs.GrabCan(prevTurning)


    turning=LineFollower.GetAction(SensorReader.leftSensorReading,SensorReader.rightSensorReading,SensorReader.dt)
    prevTurning=turning
    
    # speed=RampAdjuster.GetAction(SensorReader.inclinationAngle)

    robot.drive(speed,turning)




    

