#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor,ColorSensor
from time import time

class LinefollowBeh:

    def __init__(self,devices,pidParam,reflectionThreshold,baseSpeed) -> None:
        self.ev3=devices.ev3
        self.leftColorSensor=devices.leftColorSensor
        self.rightColorSensor=devices.rightColorSensor
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
        self.baseSpeed=baseSpeed


    def GetAction(self):
        #get time diff
        dt=time()-self.prevtime
        self.prevtime=time()

        #measure error
        if self.GetReflection(self.leftColorSensor)<self.reflectionThreshold:
            self.error=-1
        elif self.GetReflection(self.rightColorSensor)<self.reflectionThreshold:
            self.error=1
        else:
            self.error=0

        error=(self.GetReflection(self.leftColorSensor)-100)-(self.GetReflection(self.rightColorSensor)-100)

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

        #extra_speed=max(100-self.Mean(self.prev_actuators),0)
        extra_speed=0
        if self.baseSpeed<0:
            speed=self.baseSpeed-extra_speed
        else:
            speed=self.baseSpeed+extra_speed

        return (speed,self.actuators)

    def Mean(self,array):
        sum=0
        for i in range(len(array)):
            sum+=abs(array[i])
        return sum/len(array)
    
    def GetReflection(self,sensor):
        return sensor.reflection()


class Devices():
    def __init__(self,ev3,leftColorSensor,rightColorSensor) -> None:
        self.ev3=ev3
        self.leftColorSensor=leftColorSensor
        self.rightColorSensor=rightColorSensor

class PIDParam():
    def __init__(self,kp,ki,kd) -> None:
        self.kp=kp
        self.ki=ki
        self.kd=kd


if __name__=="__main__":
    from pybricks.parameters import Port,Color
    pidP=PIDParam(60,1,0.02)
    devices=Devices(EV3Brick(),ColorSensor(Port.S1),ColorSensor(Port.S2))
    linfollower=LinefollowBeh(devices,pidP,10,60)
    print(linfollower.GetAction())