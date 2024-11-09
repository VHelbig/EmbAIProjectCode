#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from LineFollowBeh import LinefollowBeh, PIDParam, Devices


from gripper import operate_gripper

# Initialize the EV3 Brick and other components
ev3 = EV3Brick()
left_motor = Motor(Port.D)
right_motor = Motor(Port.A)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

leftSensor = ColorSensor(Port.S1)
rightSensor = ColorSensor(Port.S4)
ultrasonicSensor = UltrasonicSensor(Port.S2)

# Line following setup
threshold = 13
k = 10
ki = 0
kd = 0

pidParam = PIDParam(k, ki, kd)
devices = Devices(ev3, leftSensor, rightSensor)
baseSpeed = 50
LineFollower = LinefollowBeh(devices, pidParam, threshold, baseSpeed)

# State machine variables
state = 0  
prev_distances = [0, 0, 0, 0, 0, 0, 0]
gripper_operated = False 

def Mean(array):
    return sum(abs(x) for x in array) / len(array)

def Median(array): 
    sorted_arr = sorted(array)
    return sorted_arr[len(array) // 2]

while True:
    distance = ultrasonicSensor.distance()

    if state == 0:
        speed, turning = LineFollower.GetAction()
        robot.drive(speed, turning)

        if Mean(prev_distances) - distance > 200:
            robot.stop()
            wait(500)
            ev3.speaker.beep()
            state = 1
        
    elif state == 1:
        speed, turning = LineFollower.GetAction()
        robot.drive(speed, turning)

        if Mean(prev_distances) - distance < 100:
            state = 2
            robot.stop()
            wait(1000)
        else:
            state = 0

    elif state == 2:
        if Mean(prev_distances) - distance > 10:
            robot.straight(10)

        if distance < 100 and not
gripper_operated: 
            ev3.speaker.beep()
            state = 3
            operate_gripper()  
            gripper_operated = True

    elif state == 3:
        robot.stop()

    prev_distances[1:] = prev_distances[:-1]
    prev_distances[0] = ultrasonicSensor.distance()