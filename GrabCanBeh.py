#!/usr/bin/env pybricks-micropython

from pybricks.parameters import Stop
from pybricks.tools import wait

def GoToCan(robot, distSensor, gripper_motor, speed, distance):
    prevDist=distance
    distance = distSensor.distance()
    while distance < prevDist : 
        robot.drive(speed,0)
        prevDist=distance
        distance= distSensor.distance()
    robot.stop()
    robot.straight(20)
    operate_gripper(gripper_motor)
    wait(1000)

    robot.turn(180)


def operate_gripper(gripper_motor):
    # Open the gripper
    gripper_motor.run_angle(-500, 1500, Stop.HOLD)  
    wait(2000)

    # Close the gripper
    gripper_motor.run_angle(500, 2750, Stop.HOLD)  
    wait(2000) 
