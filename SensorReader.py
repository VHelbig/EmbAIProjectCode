from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor,ColorSensor, GyroSensor, UltrasonicSensor
from pybricks.parameters import Port, Color
from time import time

class SensorReading():
    def __init__(self) -> None:

        #Initialize sensors
        self.ev3=EV3Brick()
        self.leftSensor=ColorSensor(Port.S1)
        self.rightSensor=ColorSensor(Port.S4)
        self.AngleSensor=GyroSensor(Port.S3)
        self.AngleSensor.reset_angle(0)
        self.UltraSensor=UltrasonicSensor(Port.S2)
        

        #initialize internal variables
        self.canGrabbed=False
        self.prevdistance=0
        self.prevtime=time()
        self.lastDistanceTime=0
        self.lastGreatDistanceTime=0
        self.whiteCounter=0

        #initialize outputs
        self.time=0
        self.dt=0
        self.inclinationAngle=0
        self.leftSensorReading=0
        self.rightSensorReading=0
        self.distance=0
        self.endOfLine=False
        self.potentialCan=False
        self.justUpRamp=False
        self.stillBottomFloor=True

    def GetInput(self):
        #Time related
        self.time=time()
        self.dt=self.time-self.prevtime
        self.prevtime=self.time

        #angle related
        # AngularVelo=self.AngleSensor.angle()
        # self.AngleSensor.reset_angle(0)
        # if abs(AngularVelo)>4:
        #     self.inclinationAngle+=AngularVelo
        self.inclinationAngle=self.AngleSensor.angle()

        #sensor readings
        self.leftSensorReading=self.leftSensor.reflection()
        self.rightSensorReading=self.rightSensor.reflection()

        #ultraSonic sensor
        self.prevdistance=self.distance
        self.distance=self.UltraSensor.distance()

        #just passed angle
        # if self.inclinationAngle<-25:
        #     self.lastAngleTime=self.time
        # if self.time-self.lastAngleTime<2:
        #     self.justUpRamp=True
        # else:
        #     self.justUpRamp=False
        if self.distance>1200:
            self.lastDistanceTime=self.time
        if self.time-self.lastDistanceTime<8:
            self.justUpRamp=True
        else:
            self.justUpRamp=False

        #end of Line
        if not self.justUpRamp:
            if self.leftSensorReading*0.5+self.rightSensorReading*0.5 > 17:
                if self.whiteCounter>1:
                    self.endOfLine=True
                else:
                    self.whiteCounter+=self.dt
            else:
                self.whiteCounter=0
                self.endOfLine=False

        #still bottom floor
        if self.distance>1100:
            self.lastGreatDistanceTime=self.time
        if self.time-self.lastGreatDistanceTime<10:
            self.stillBottomFloor=True
        else:
            # if self.stillBottomFloor:
            #     # self.ev3.speaker.beep(1000,100)
            self.stillBottomFloor=False

        #potential Can
        if not self.stillBottomFloor:
            d_diff=self.distance-self.prevdistance
            if self.distance>90 and self.distance<250:
                if d_diff<-10 and d_diff>-150:
                    self.potentialCan=True
                else:
                    self.potentialCan=False
            else:
                self.potentialCan=False

    
                
if __name__ == "__main__":
    SensorReader=SensorReading()
    print(SensorReader.GetInput())