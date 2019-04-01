# import setup_path
import airsim
import cv2
import time
import random
from random import randint, uniform
import os 
import numpy as np

'''Random한 x,y 값 구하기 car 1'''
a=random.randint(1,3)

if a==1:
    x=random.choice([random.uniform(125,131),random.uniform(-3,3),random.uniform(-131,-125)])
    y=random.uniform(-132,130)

elif a==2:
    x=random.choice([random.uniform(-7,-121),random.uniform(7,74),random.uniform(88,121)])
    y=random.choice([random.uniform(-4,2),random.uniform(-132,-126),random.uniform(124,130)])

elif a==3:
    x=random.uniform(78,84)
    y=random.uniform(-132,2)

'''random position for car2'''

b=random.randint(1,3)

if b==1:
    i=random.choice([random.uniform(125,131),random.uniform(-3,3),random.uniform(-131,-125)])
    j=random.uniform(-132,130)

elif b==2:
    i=random.choice([random.uniform(-7,-121),random.uniform(7,74),random.uniform(88,121)])
    j=random.choice([random.uniform(-4,2),random.uniform(-132,-126),random.uniform(124,130)])

elif b==3:
    i=random.uniform(78,84)
    j=random.uniform(-132,2)


'''Random한 car orientation 구하기'''
c=random.randint(0,360) 
d=random.randint(0,360) 

print (x,y,b)

# connect to the AirSim simulator 
client = airsim.CarClient(ip="192.168.0.51")
client.confirmConnection()
client.enableApiControl(True, "Car1")
client.enableApiControl(True, "Car2")

car_controls1 = airsim.CarControls()
car_controls2 = airsim.CarControls()


start = time.time()

car_controls1.position = airsim.Vector3r(x,y,-1)
car_controls1.heading = airsim.utils.to_quaternion(0,0,c)
car_controls1.pose = airsim.Pose(car_controls1.position,car_controls1.heading)

car_controls2.position = airsim.Vector3r(i,j,-1)
car_controls2.heading = airsim.utils.to_quaternion(0,0,d)
car_controls2.pose = airsim.Pose(car_controls2.position,car_controls2.heading)

client.simSetVehiclePose(car_controls1.pose, True,vehicle_name="Car1")
client.simSetVehiclePose(car_controls2.pose, True,vehicle_name="Car2")


