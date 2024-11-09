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
from pybricks.ev3devices import Motor,ColorSensor, GyroSensor, UltrasonicSensor
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from time import time
from LineFollowBeh import LinefollowBeh,PIDParam,Devices
from RampBeh import RampBeh,RampParam
from gripper import operate_gripper

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the motors
left_motor = Motor(Port.D)
right_motor = Motor(Port.A)

#Initialize sensors
leftSensor=ColorSensor(Port.S1)
rightSensor=ColorSensor(Port.S4)
AngleSensor=GyroSensor(Port.S3)
AngleSensor.reset_angle(0)
UltraSensor=UltrasonicSensor(Port.S2)

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
devices=Devices(ev3,leftSensor,rightSensor)
rampParams=RampParam(speedUpFactor,speedDownFactor)


LineFollower=LinefollowBeh(devices,pidParam,threshold,baseSpeed)
RampAdjuster=RampBeh(AngleSensor,rampParams,baseSpeed)
speed=baseSpeed

prevdistance=0
once=False
def GrabCan():
    robot.stop()
    robot.turn(-10)
    
    global once
    if once==True:
        return
    distance=UltraSensor.distance()
    ev3.speaker.beep(500,200)
    while(distance>40):
        robot.drive(speed,0)
        distance=UltraSensor.distance()
    robot.stop()
    robot.straight(20)
    ev3.speaker.beep(500,100)
    operate_gripper()
    robot.turn(230)
    robot.stop()
    once=True

while True:
    distance=UltraSensor.distance()
    d_diff=distance-prevdistance
    prevdistance=distance
    

    #ev3.screen.print(distance)
    if distance>90 and distance<200:
        if d_diff<-10:
            GrabCan()

    turning=LineFollower.GetAction()
    
    #speed=RampAdjuster.GetAction()

    robot.drive(speed,turning)




    

