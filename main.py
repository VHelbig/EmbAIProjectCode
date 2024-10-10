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
from pybricks.ev3devices import Motor,ColorSensor
from pybricks.parameters import Port, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from time import time
from LineFollowBeh import LinefollowBeh,PIDParam,Devices

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the motors
left_motor = Motor(Port.D)
right_motor = Motor(Port.A)

#Initialize sensors
leftSensor=ColorSensor(Port.S1)
rightSensor=ColorSensor(Port.S4)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

threshold=13
k=10
ki=0
kd=0

pidParam=PIDParam(k,ki,kd)
devices=Devices(ev3,leftSensor,rightSensor)
baseSpeed=50

LineFollower=LinefollowBeh(devices,pidParam,threshold,baseSpeed)

while True:
    (speed,turning)=LineFollower.GetAction()
    robot.drive(speed,turning)

    

