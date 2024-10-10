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

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the motors
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)

#Initialize sensors
leftSensor=ColorSensor(Port.S1)
rightSensor=ColorSensor(Port.S4)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

def GetReflection(sensor):
    #ambient=sensor.ambient()
    return sensor.reflection()#-ambient

def mean(array):
    sum=0
    for i in range(len(array)):
        sum+=abs(array[i])
    return sum/len(array)

threshold=13
#treshold = (leftSensor + rightSensor) / 2;


k=100 #2
ki=0 #1 maybe 
kd=0.02 #0.02



error_int=0

prevtime=time()
preverror=0

prev_actuators=[0,0,0,0,0,0,0]

while True:
    dt=time()-prevtime
    prevtime=time()

    if GetReflection(leftSensor)<threshold:
        error=1
    elif GetReflection(rightSensor)<threshold:
        error=-1
    else:
        error=0
    

    
    error_int+=error*dt
    
    error_dev=(error-preverror)/dt

    actuators=k*(error+ki*error_int+kd*error_dev)
    prev_actuators[1:]=prev_actuators[:-1]
    prev_actuators[0]=actuators

    

    extra_speed=100-mean(prev_actuators)

    robot.drive(30+max(extra_speed,0),actuators)
    ev3.screen.print(100-mean(prev_actuators))

    preverror=error

    

