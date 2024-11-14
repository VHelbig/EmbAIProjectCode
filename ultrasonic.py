#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from LineFollowBeh import LinefollowBeh, PIDParam, Devices
from pybricks.tools import StopWatch

# Import the gripper operation function
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

# Detection variables
gripper_operated = False 
distance_history = [1000] * 5  
can_detection_en = False
timer = StopWatch();

def can_detection_en():
    can_detection_en = True
    timer.reset()


def median(array):
    sorted_array = sorted(array)
    return sorted_array[len(array) // 2]

while True:
    # Line following action
    speed, turning = LineFollower.GetAction()
    robot.drive(speed, turning)

    # Object detection logic
    distance = ultrasonicSensor.distance()
    distance_history.pop(0)
    distance_history.append(distance)

    median_distance = median(distance_history)

    if can_detection_en and timer.time() > 500:
        can_detection_en = False

    if can_detection_en and not gripper_operated and 50 < median_distance < 100:

        while median_distance > 50:
            robot.straight(10)
            distance = ultrasonicSensor.distance()
            distance_history.pop(0)
            distance_history.append(distance)
            median_distance = median(distance_history)

        # Stop and operate the gripper once
        robot.stop()
        operate_gripper()
        gripper_operated = True
        wait(2000) 

        # Resume line following
        robot.drive(speed, turning)

    if median_distance > 100:  # Object is out of range consistently
        gripper_operated = False  # Ready to detect the next object