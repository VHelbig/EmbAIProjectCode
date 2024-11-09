#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor,ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from time import time
from LineFollowBeh import LinefollowBeh,PIDParam,Devices

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the motors
left_motor = Motor(Port.D)
right_motor = Motor(Port.A)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

# Initialize sensors
leftSensor = ColorSensor(Port.S1)
rightSensor = ColorSensor(Port.S4)
ultrasonicSensor = UltrasonicSensor(Port.S2)

# For following the line
threshold = 13
k = 10
ki = 0
kd = 0

pidParam = PIDParam(k,ki,kd)
devices = Devices(ev3,leftSensor,rightSensor)
baseSpeed = 50

LineFollower=LinefollowBeh(devices,pidParam,threshold,baseSpeed)

# For searching the can
prev_distances = [0,0,0,0,0,0,0]

def Mean(array):
    sum = 0
    for i in range(len(array)):
        sum += abs(array[i])
    return sum/len(array)

def Median(array): # !! fonctionne pour tableau de taille impaire (ici 7)
    sorted_arr = sorted(array)
    return sorted_arr[len(array) // 2]
    
state = 0 # 0: Following the can + looking for the can 
          # 1: Maybe the can
          # 2: Go to the can
          # 3: Stop

# tests
k = 0

while True:

    distance = ultrasonicSensor.distance()

    if state == 0:
        (speed, turning) = LineFollower.GetAction()
        robot.drive(speed, turning)

        if (Mean(prev_distances) - distance) > 200 :
            #ev3.screen.print(prev_distances)
            #ev3.screen.print(distance)
            robot.stop()
            wait(500)
            ev3.speaker.beep()
            state = 1
        
    elif state == 1:
        (speed, turning) = LineFollower.GetAction()
        robot.drive(speed, turning)

        if (Mean(prev_distances) - distance) < 100 :
            state = 2
            robot.stop()
            wait(1000)
        else:
            state = 0

    elif state == 2:
        if (Mean(prev_distances) - distance) > 10 :
            robot.straight(10)

        if distance < 100 : 
            ev3.speaker.beep()
            state = 2

    elif state == 3:
        robot.stop()

    prev_distances[1:] = prev_distances[:-1]
    prev_distances[0] = ultrasonicSensor.distance()