#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Stop
from pybricks.tools import wait

gripper_motor = Motor(Port.B)


def operate_gripper():
    # Open the gripper
    gripper_motor.run_angle(-500, 1500, Stop.HOLD)  
    wait(2000)  

    # Close the gripper
    gripper_motor.run_angle(500, 2450, Stop.HOLD)  
    wait(2000)  
