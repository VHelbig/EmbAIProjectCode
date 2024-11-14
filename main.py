#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait
from LineFollowBehP import LinefollowBeh,PIDParam,Devices, RampParam
from GrabCanBeh import GoToCan

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the motors
left_motor = Motor(Port.D)
right_motor = Motor(Port.A)
gripper_motor = Motor(Port.B)

#Initialize sensors
leftSensor=ColorSensor(Port.S1)
rightSensor=ColorSensor(Port.S4)
angleSensor=GyroSensor(Port.S3)
angleSensor.reset_angle(0)
ultraSensor=UltrasonicSensor(Port.S2)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)


# Settings (line following + ramp)
threshold=13
k=10
ki=0
kd=0
baseSpeed=50
speedUpFactor=2
speedDownFactor=0.5


pidParam=PIDParam(k,ki,kd)
rampParams=RampParam(speedUpFactor,speedDownFactor,baseSpeed)
devices=Devices(leftSensor,rightSensor,angleSensor)

LineFollower=LinefollowBeh(devices,pidParam,rampParams,threshold)

# Variables
speed=baseSpeed
prevdistance=0

while True:

    distance=ultraSensor.distance()
    d_diff=distance-prevdistance
    prevdistance=distance

    if d_diff<-100:
        ev3.speaker.beep()
        if prevdistance<1500:
            robot.stop()
            wait(1000)
            sens=turning/abs(turning)
            #"robot.turn(sens*5)
            if abs(ultraSensor.distance() - prevdistance) < 10:
                ev3.speaker.beep()
                robot.turn(-sens*5)
                robot.stop()
                wait(1000)
                GoToCan(robot,ultraSensor,gripper_motor,speed,distance)


    speed, turning = LineFollower.GetAction()
    robot.drive(speed, turning)

"""

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

"""