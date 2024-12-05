#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor,ColorSensor
from time import time

class LinefollowBeh:

    def __init__(self,pidParam,reflectionThreshold) -> None:
        self.reflectionThreshold=reflectionThreshold
        self.kp=pidParam.kp
        self.ki=pidParam.ki
        self.kd=pidParam.kd
        self.int_sat_value=2
        self.error=0
        self.preverror=0
        self.prev_actuators=[0,0,0,0,0,0,0]
        self.actuators=0
        self.error_int=0
        self.error_dev=0
        self.previous_devs=[0,0,0,0,0,0]


    def GetAction(self,leftSensorReading,rightSensorReading,dt):
        #measure error
        self.error=(leftSensorReading-rightSensorReading)

        #integral and derivative    
        self.error_int+=self.error*dt
        if abs(self.error_int)>=self.int_sat_value:
            if self.error_int<0:
                self.error_int=-self.int_sat_value
            else:
                self.error_int=self.int_sat_value

        # for i in range(1,len(self.previous_devs)):
        #     self.previous_devs[i]=self.previous_devs[i-1]
        # self.previous_devs[0]=(self.error-self.preverror)/dt
        # self.error_dev=self.Mean(self.previous_devs)

        self.error_dev=(self.error-self.preverror)/dt

        #compute control signal
        self.actuators=self.kp*(self.error+self.ki*self.error_int+self.kd*self.error_dev)
        self.prev_actuators[1:]=self.prev_actuators[:-1]
        self.prev_actuators[0]=self.actuators

        return self.actuators

    def Mean(self,array):
        sum=0
        for i in range(len(array)):
            sum+=abs(array[i])
        return sum/len(array)


class PIDParam():
    def __init__(self,kp,ki,kd) -> None:
        self.kp=kp
        self.ki=ki
        self.kd=kd


if __name__=="__main__":
    from pybricks.parameters import Port,Color
    pidP=PIDParam(60,1,0.02)
    linfollower=LinefollowBeh(pidP,10)
    print(linfollower.GetAction(10,10,10))