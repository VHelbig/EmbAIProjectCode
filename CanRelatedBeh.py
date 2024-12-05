from pybricks.robotics import DriveBase
from SensorReader import SensorReading
from gripper import operate_gripper
from math import copysign

class CanRelatedBeh():
    def __init__(self,robot,SensorReader,baseSpeed) -> None:
        self.robot=robot
        self.SensorReader=SensorReader
        self.baseSpeed=baseSpeed

    def SearchCan(self):
        self.robot.straight(-self.baseSpeed*2)
        self.robot.turn(50)
        self.robot.stop()    
        d_diff=0
        prevdistance=self.SensorReader.UltraSensor.distance()
        condition=True
        while(condition):
            distance=self.SensorReader.UltraSensor.distance()
            d_diff=distance-prevdistance
            condition=(d_diff>-20 and d_diff>-200) and distance <150
            prevdistance=distance
            self.robot.turn(-1)
        self.GrabCan(-1)

    def GrabCan(self,prevturnning):
        self.robot.stop()
        self.robot.turn(copysign(10,prevturnning))
        self.robot.stop()
        distance=self.SensorReader.UltraSensor.distance()
        while(distance>50):
            self.robot.straight(10)
            distance=self.SensorReader.UltraSensor.distance()
        self.robot.straight(20)
        operate_gripper()
        self.robot.turn(250)
        self.robot.stop()
        self.SensorReader.canGrabbed=True