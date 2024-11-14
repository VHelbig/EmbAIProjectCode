#!/usr/bin/env pybricks-micropython

from time import time

class LinefollowBeh:

    def __init__(self,devices,pidParam,rampParams,reflectionThreshold) -> None:
        self.leftColorSensor=devices.leftColorSensor
        self.rightColorSensor=devices.rightColorSensor
        self.angleSensor=devices.angleSensor

        self.reflectionThreshold=reflectionThreshold

        self.kp=pidParam.kp
        self.ki=pidParam.ki
        self.kd=pidParam.kd

        self.int_sat_value=2
        self.prevtime=time()-0.001
        self.error=0
        self.preverror=0
        self.prev_actuators=[0,0,0,0,0,0,0]
        self.actuators=0
        self.error_int=0
        self.error_dev=0

        self.speedUpFactor=rampParams.speedUpFactor
        self.speedDownFactor=rampParams.speedDownFactor
        self.baseSpeed=rampParams.baseSpeed

    def GetAction(self):
        #get time diff
        dt=time()-self.prevtime
        self.prevtime=time()

        #measure error
        self.error=(self.GetReflection(self.leftColorSensor))-(self.GetReflection(self.rightColorSensor))

        #integral and derivative    
        self.error_int+=self.error*dt
        if abs(self.error_int)>=self.int_sat_value:
            if self.error_int<0:
                self.error_int=-self.int_sat_value
            else:
                self.error_int=self.int_sat_value

        self.error_dev=(self.error-self.preverror)/dt

        #compute control signal
        self.actuators=self.kp*(self.error+self.ki*self.error_int+self.kd*self.error_dev)
        self.prev_actuators[1:]=self.prev_actuators[:-1]
        self.prev_actuators[0]=self.actuators

        angle=self.angleSensor.angle()
        if angle<-20:
            speed=self.baseSpeed*self.speedUpFactor
        elif angle >20:
            speed=self.baseSpeed*self.speedDownFactor
        else:
            speed=self.baseSpeed
        return (speed, self.actuators)
    
    def GetReflection(self,sensor):
        return sensor.reflection()


class Devices():
    def __init__(self,leftColorSensor,rightColorSensor,angleSensor) -> None:
        self.leftColorSensor=leftColorSensor
        self.rightColorSensor=rightColorSensor
        self.angleSensor=angleSensor
        self.angleSensor.reset_angle(0)

class PIDParam():
    def __init__(self,kp,ki,kd) -> None:
        self.kp=kp
        self.ki=ki
        self.kd=kd

class RampParam():
    def __init__(self,speedUpFactor,speedDownFactor,baseSpeed) -> None:
        self.speedUpFactor=speedUpFactor
        self.speedDownFactor=speedDownFactor
        self.baseSpeed=baseSpeed


if __name__=="__main__":
    from pybricks.hubs import EV3Brick
    from pybricks.ev3devices import Motor, ColorSensor, GyroSensor
    from pybricks.parameters import Port
    from pybricks.robotics import DriveBase
    pidP=PIDParam(10,0,0)
    rampP=RampParam(2,0.5,50)
    devices=Devices(ColorSensor(Port.S1),ColorSensor(Port.S4),GyroSensor(Port.S3))
    linfollower=LinefollowBeh(devices,pidP,rampP,13)

    ev3 = EV3Brick()
    left_motor = Motor(Port.D)
    right_motor = Motor(Port.A)
    robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

    while True:
        speed, turning = linfollower.GetAction()
        robot.drive(speed, turning)
