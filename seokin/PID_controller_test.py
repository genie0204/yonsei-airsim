#-*- coding: utf-8 -*-
from __future__ import print_function, absolute_import, division

import time

import sys

import math
# import setup_path 
import airsim
import numpy as np

class PID:

    def __init__(self, dt=0.05, Kp=0, Ki=0, Kd=0):

        self.dt = dt

        self.Kp = Kp

        self.Ki = Ki

        self.Kd = Kd

        self.reset()

    def feedback(self, err):

        if type(self.err_p) == type(None):

            self.err_p = err

        self.errsum += err*self.dt

        d_err = (err-self.err_p)/self.dt

        self.err_p = err



        return self.Kp*err+self.Ki*self.errsum+self.Kd*d_err



    def reset(self):

        self.err_p = None

        self.errsum = 0.0

class CarController:
    def __init__(self, client, car_name):
        self.client = client
        self.client.enableApiControl(True, car_name)

        self.car_name = car_name

        self.car_controls = airsim.CarControls()
        self.client.setCarControls(self.car_controls)

        self.coord_index = 1
        self.coordinates = []

        self.pid_distance_staright= PID(dt=0.05, Kp=1, Ki=0, Kd=0.001)
        self.pid_degrees_straight = PID(dt=0.05, Kp=0.1, Ki=0, Kd=0.001)
        self.pid_distance_curve = PID(dt=0.05, Kp=0.1, Ki=0, Kd=0.001)
        self.pid_degrees_curve = PID(dt=0.05, Kp=3, Ki=0, Kd=1)
        self.pid_distance_escape= PID(dt=0.05, Kp=0.2, Ki=0.001, Kd=0.001)
        self.pid_degrees_escape = PID(dt=0.05, Kp=3, Ki=0, Kd=1)

    def setPosition(self, point, direction):
        self.car_controls.position = airsim.Vector3r(point[0],point[1],0)
        self.car_controls.heading = airsim.utils.to_quaternion(0,0,direction)
        self.car_controls.pose = airsim.Pose(self.car_controls.position,self.car_controls.heading)

        self.client.simSetVehiclePose(self.car_controls.pose, True,vehicle_name= self.car_name)

    def moveCar(self, distance, degree):
        # car_controls = airsim.CarControls()
        self.car_controls.throttle = distance
        self.car_controls.steering = degree
       
        self.client.setCarControls(self.car_controls)
        car_state = self.client.getCarState()
        pos_x = car_state.kinematics_estimated.position.x_val
        pos_y = car_state.kinematics_estimated.position.y_val

        return pos_x, pos_y

    def stopCar(self):
        while True:
            self.car_controls.throttle = 0
            self.car_controls.steering = 0

            self.client.setCarControls(self.car_controls)

            if self.client.getCarState().speed < 0.01:
                break

        return True

    def compute_1(self, coordinates):
        if len(self.coordinates) == 0:
            self.coordinates = coordinates
        
        car_state = self.client.getCarState()

        pos_x = car_state.kinematics_estimated.position.x_val
        pos_y = car_state.kinematics_estimated.position.y_val
        target = airsim.Vector3r(coordinates[self.coord_index][0],coordinates[self.coord_index][1],0)
        target_1 = airsim.Vector3r(coordinates[self.coord_index+1][0],coordinates[self.coord_index+1][1],0)
           
        target_x = target.x_val
        target_1_x = target_1.x_val
        target_y = target.y_val
        target_1_y = target_1.y_val
        print ("target_1", target_1_x ,target_1_y) 
        
        
        print("target", target_x,target_y)
        print("pos", pos_x, pos_y)
        err_x = target_x - pos_x
        err_x_1 = target_1_x - pos_x
        err_y = target_y - pos_y
        err_y_1 = target_1_y - pos_y
        err_distance = math.sqrt(err_x ** 2 + err_y ** 2)
        # print (err_distance)
       

        if self.coord_index == len(coordinates):
            done = True
            self.coordinates = []
        else:
            done = False


        #각도  + - 튀기는 것 수정한 부분입니다
        orientation_z = car_state.kinematics_estimated.orientation.z_val * math.pi
        if orientation_z < 0 :
            orientation_z_convert = math.pi*2 + orientation_z
        else :
            orientation_z_convert = orientation_z
        target_degree = math.atan2(err_y_1,err_x_1)
        if target_degree < 0 :
            target_degree_convert = math.pi*2 + target_degree
        else :

            target_degree_convert = target_degree
        err_degree = target_degree_convert - orientation_z_convert
        if err_degree > math.pi : 
            err_degree_convert = err_degree - math.pi*2
        elif err_degree < -math.pi :
            err_degree_convert = math.pi*2 + err_degree
        else :
            err_degree_convert = err_degree
        
        

        # 짜잘한 움직임을 제한하기 위해 0도 근처 값 제한
        #if abs(err_degree_convert) < math.pi /30:
        #    err_degree_convert = 0
    
        try:
            # print(math.degrees(err_degree), math.degrees(target_degree), math.degrees(orientation_z))
            coordinates_degree = math.degrees(math.atan2(coordinates[self.coord_index+3
            ][1]-coordinates[self.coord_index+2][1],
                                                coordinates[self.coord_index+3][0]-coordinates[self.coord_index+2][0]))
        except:
            coordinates_degree = 0

        
    
        distance = self.pid_distance_staright.feedback(err_distance)
        degree = self.pid_degrees_straight.feedback(err_degree_convert)
        if (err_distance<0.3) :
            self.coord_index += 1
        
        
        
        self.moveCar(distance, degree)
        d = self.getDistanceFromStraightLine((pos_x, pos_y))

        print ("target", math.degrees(target_degree_convert))
        print ("orentation", math.degrees(orientation_z_convert))
        print ("err_degree", math.degrees(err_degree_convert))
        print ("err_distance", err_distance)
        print ("d", d)
        
        print("===============================")
        return done, d , coordinates_degree, err_degree_convert

    def compute_2(self, coordinates):
        if len(self.coordinates) == 0:
            self.coordinates = coordinates
        
        car_state = self.client.getCarState()

        pos_x = car_state.kinematics_estimated.position.x_val
        pos_y = car_state.kinematics_estimated.position.y_val
        target = airsim.Vector3r(coordinates[self.coord_index][0],coordinates[self.coord_index][1],0)
        target_1 = airsim.Vector3r(coordinates[self.coord_index+3][0],coordinates[self.coord_index+3][1],0)    
        target_x = target.x_val
        target_1_x = target_1.x_val
        target_y = target.y_val
        target_1_y = target_1.y_val
        print("++++++++++++++++++++curve++++++++++++++++++")
        
        print("target", target_x,target_y)
        print("pos", pos_x, pos_y)
        err_x = target_x - pos_x
        err_x_1 = target_1_x - pos_x
        err_y = target_y - pos_y
        err_y_1 = target_1_y - pos_y
        err_distance = math.sqrt(err_x ** 2 + err_y ** 2)
        # print (err_distance)
       

        if self.coord_index == len(coordinates):
            done = True
            self.coordinates = []
        else:
            done = False


        #각도  + - 튀기는 것 수정한 부분입니다
        orientation_z = car_state.kinematics_estimated.orientation.z_val * math.pi
        if orientation_z < 0 :
            orientation_z_convert = math.pi*2 + orientation_z
        else :
            orientation_z_convert = orientation_z
        target_degree = math.atan2(err_y_1,err_x_1)
        if target_degree < 0 :
            target_degree_convert = math.pi*2 + target_degree
        else :

            target_degree_convert = target_degree
        err_degree = target_degree_convert - orientation_z_convert
        if err_degree > math.pi : 
            err_degree_convert = err_degree - math.pi*2
        elif err_degree < -math.pi :
            err_degree_convert = math.pi*2 + err_degree
        else :
            err_degree_convert = err_degree
        
        

        # 짜잘한 움직임을 제한하기 위해 0도 근처 값 제한
        #if abs(err_degree_convert) < math.pi /30:
        #    err_degree_convert = 0
    
        try:
            # print(math.degrees(err_degree), math.degrees(target_degree), math.degrees(orientation_z))
            coordinates_degree = math.degrees(math.atan2(coordinates[self.coord_index+5][1]-coordinates[self.coord_index+4][1],
                                                coordinates[self.coord_index+5][0]-coordinates[self.coord_index+4][0]))
        except:
            coordinates_degree = 0

        distance = self.pid_distance_curve.feedback(err_distance)
        degree = self.pid_degrees_curve.feedback(err_degree_convert)
        if (err_distance<2 ) :
            self.coord_index += 1
                
            
        
        
        

        self.moveCar(distance, degree)
        d = self.getDistanceFromStraightLine((pos_x, pos_y))

        print ("target", math.degrees(target_degree_convert))
        print ("orentation", math.degrees(orientation_z_convert))
        print ("err_degree", math.degrees(err_degree_convert))
        print ("err_distance", err_distance)
        print ("d", d)
        print("++++++++++++++++++++curve++++++++++++++++++")
        return done, d, coordinates_degree , err_degree_convert  

    def compute_3(self, coordinates):
        if len(self.coordinates) == 0:
            self.coordinates = coordinates
        
        car_state = self.client.getCarState()

        pos_x = car_state.kinematics_estimated.position.x_val
        pos_y = car_state.kinematics_estimated.position.y_val
        target = airsim.Vector3r(coordinates[self.coord_index][0],coordinates[self.coord_index][1],0)
        target_1 = airsim.Vector3r(coordinates[self.coord_index+5][0],coordinates[self.coord_index+5][1],0)    
        target_x = target.x_val
        target_1_x = target_1.x_val
        target_y = target.y_val
        target_1_y = target_1.y_val
        print("ddddddddddddddddddddddddddddd")
        
        print("target", target_x,target_y)
        print("pos", pos_x, pos_y)
        err_x = target_x - pos_x
        err_x_1 = target_1_x - pos_x
        err_y = target_y - pos_y
        err_y_1 = target_1_y - pos_y
        err_distance = math.sqrt(err_x ** 2 + err_y ** 2)
        # print (err_distance)
       

        if self.coord_index == len(coordinates):
            done = True
            self.coordinates = []
        else:
            done = False


        #각도  + - 튀기는 것 수정한 부분입니다
        orientation_z = car_state.kinematics_estimated.orientation.z_val * math.pi
        if orientation_z < 0 :
            orientation_z_convert = math.pi*2 + orientation_z
        else :
            orientation_z_convert = orientation_z
        target_degree = math.atan2(err_y_1,err_x_1)
        if target_degree < 0 :
            target_degree_convert = math.pi*2 + target_degree
        else :

            target_degree_convert = target_degree
        err_degree = target_degree_convert - orientation_z_convert
        if err_degree > math.pi : 
            err_degree_convert = err_degree - math.pi*2
        elif err_degree < -math.pi :
            err_degree_convert = math.pi*2 + err_degree
        else :
            err_degree_convert = err_degree
        
        

        # 짜잘한 움직임을 제한하기 위해 0도 근처 값 제한
        #if abs(err_degree_convert) < math.pi /30:
        #    err_degree_convert = 0
    
        try:
            # print(math.degrees(err_degree), math.degrees(target_degree), math.degrees(orientation_z))
            coordinates_degree = math.degrees(math.atan2(coordinates[self.coord_index+3][1]-coordinates[self.coord_index+2][1],
                                                coordinates[self.coord_index+3][0]-coordinates[self.coord_index+2][0]))
        except:
            coordinates_degree = 0

        
        distance = self.pid_distance_escape.feedback(err_distance)
        degree = self.pid_degrees_escape.feedback(err_degree_convert)
        if (err_distance<0.8 ) :
            self.coord_index += 1
                
            
        
        
        

        self.moveCar(distance, degree)
        d = self.getDistanceFromStraightLine((pos_x, pos_y))

        print ("target", math.degrees(target_degree_convert))
        print ("orentation", math.degrees(orientation_z_convert))
        print ("err_degree", math.degrees(err_degree_convert))
        print ("err_distance", err_distance)
        print ("d", d)
        print("ddddddddddddddddddddddddddddddddddddddddddddddd")
        return done, d, coordinates_degree  ,err_degree_convert 






    def getDistanceFromStraightLine(self, car_pos):
        p1 = self.coordinates[self.coord_index]
        if self.coord_index < len(self.coordinates)-1:
            p2 = self.coordinates[self.coord_index+1]

            slope = self.getSlope(p1, p2)
            yintercept = self.getYintercept(slope, p1)

            if slope is not None:
                return abs(slope*car_pos[0] - car_pos[1] + yintercept) / math.sqrt(slope**2+1)
            else:
                # Vertical
                return abs(car_pos[0] - p1[0])
        
        else:
            return math.sqrt((car_pos[0]-p1[0])**2+(car_pos[1]-p1[1])**2)


            
    def getSlope(self, p1, p2):
        '''Get the slope of a line segment'''
        (x1, y1), (x2, y2) = p1, p2
        try:
            return (float(y2)-y1)/(float(x2)-x1)
        except ZeroDivisionError:
            # line is vertical
            return None

    def getYintercept(self, slope, p1):
        '''Get the y intercept of a line segment'''
        if slope != None:
            x, y = p1
            return y - slope * x
        else:
            return None

    
            



