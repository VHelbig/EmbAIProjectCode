#!/usr/bin/env pybricks-micropython

from pybricks.ev3devices import GyroSensor

class RampBeh:

    def __init__(self,rampParams,baseSpeed) -> None:
        self.speedUpFactor=rampParams.speedUpFactor
        self.speedDownFactor=rampParams.speedDownFactor
        self.baseSpeed=baseSpeed

    def GetAction(self,anglularVelo):
        if anglularVelo<-25:
            speed=self.baseSpeed*self.speedUpFactor
        elif anglularVelo >25:
            speed=self.baseSpeed*self.speedDownFactor
        else:
            speed=self.baseSpeed
        return speed


class RampParam():
    def __init__(self,speedUpFactor,speedDownFactor) -> None:
        self.speedUpFactor=speedUpFactor
        self.speedDownFactor=speedDownFactor


if __name__=="__main__":
    from pybricks.parameters import Port
    rampParams=RampParam(2,0.5)
    rampBeh=RampBeh(rampParams,100)
    print(rampBeh.GetAction(10))