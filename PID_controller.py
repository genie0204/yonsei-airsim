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
        self.client.setCarControls(self.car_controls, vehicle_name=self.car_name)

        self.coord_index = 1
        self.coordinates = []

        
        self.pid_degrees_straight = PID(dt=0.05, Kp=0.5, Ki=0, Kd=0.001)
        self.pid_degrees_curve = PID(dt=0.05, Kp=5, Ki=0, Kd=1)

        self.pos_xs = []
        self.pos_ys = []
      
    def storePos(self, x, y):
        self.pos_xs.append(x)
        self.pos_ys.append(y)

    def setPosition(self, point, direction):
        self.car_controls.position = airsim.Vector3r(point[0],point[1],0)
        self.car_controls.heading = airsim.utils.to_quaternion(0,0,direction)
        self.car_controls.pose = airsim.Pose(self.car_controls.position,self.car_controls.heading)

        self.client.simSetVehiclePose(self.car_controls.pose, True, vehicle_name= self.car_name)

    def getPosition(self):
        car_state = self.client.getCarState(vehicle_name=self.car_name)

        pos_x = car_state.kinematics_estimated.position.x_val
        pos_y = car_state.kinematics_estimated.position.y_val

        return pos_x, pos_y

    def moveCar(self, distance, degree):
        # car_controls = airsim.CarControls()
        self.car_controls.throttle = distance
        self.car_controls.steering = degree
       
        self.client.setCarControls(self.car_controls, vehicle_name=self.car_name)
        car_state = self.client.getCarState(vehicle_name=self.car_name)
        # print(self.car_name, car_state)
        pos_x = car_state.kinematics_estimated.position.x_val
        pos_y = car_state.kinematics_estimated.position.y_val

        return pos_x, pos_y

    def stopCar(self):
        while True:
            self.car_controls.throttle = 0
            self.car_controls.steering = 0

            self.client.setCarControls(self.car_controls, vehicle_name=self.car_name)

            # if self.client.getCarState(vehicle_name=self.car_name).speed < 0.01:
            if self.client.getCarState(vehicle_name=self.car_name).speed < 0.1:
                break

        return True

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

    def keepTrack(self, coordinates):
        car_state = self.client.getCarState(vehicle_name=self.car_name)

        pos_x = car_state.kinematics_estimated.position.x_val
        pos_y = car_state.kinematics_estimated.position.y_val
        # print(pos_x, pos_y)
        self.storePos(pos_x, pos_y)
        target = airsim.Vector3r(coordinates[self.coord_index][0],coordinates[self.coord_index][1],0)
        target_x = target.x_val
        target_y = target.y_val
        err_x = target_x - pos_x
        err_y = target_y - pos_y
        err_distance = math.sqrt(err_x ** 2 + err_y ** 2)

        if (err_distance<0.5) :
            self.coord_index += 1
    
    def compute_straight(self, coordinates):

        # print ('==================straigth==================')
        if len(self.coordinates) == 0:
            self.coordinates = coordinates

        else:
            done = False    
        
        car_state = self.client.getCarState(vehicle_name=self.car_name)

        pos_x = car_state.kinematics_estimated.position.x_val
        pos_y = car_state.kinematics_estimated.position.y_val
        # print(pos_x, pos_y)
        self.storePos(pos_x, pos_y)
        target = airsim.Vector3r(coordinates[self.coord_index][0],coordinates[self.coord_index][1],0)
        target_x = target.x_val
        target_y = target.y_val
        err_x = target_x - pos_x
        err_y = target_y - pos_y
        err_distance = math.sqrt(err_x ** 2 + err_y ** 2)
        
       

        if self.coord_index == len(coordinates):
            done = True
            self.coordinates = []
        else:
            done = False

        #각도  + - 튀기는 것 수정한 부분입니다
        x = car_state.kinematics_estimated.orientation.x_val
        y = car_state.kinematics_estimated.orientation.y_val
        z = car_state.kinematics_estimated.orientation.z_val
        w = car_state.kinematics_estimated.orientation.w_val
        orientation_z = math.atan2(2*(x*y+z*w),x*x-y*y-z*z+w*w)
        
        if orientation_z < 0 :
            orientation_z_convert = math.pi*2 + orientation_z
        else :
            orientation_z_convert = orientation_z
        target_degree = math.atan2(err_y,err_x)
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
        if abs(err_degree_convert) < math.pi /24:
            err_degree_convert = 0
    
        
        coordinates_degree_1 = math.degrees(math.atan2(coordinates[self.coord_index+6][1]
        -coordinates[self.coord_index+5][1],coordinates[self.coord_index+6][0]-coordinates[self.coord_index+5][0]))
        
        coordinates_degree_2 = math.degrees(math.atan2(coordinates[self.coord_index+7][1]
        -coordinates[self.coord_index+6][1],coordinates[self.coord_index+7][0]-coordinates[self.coord_index+6][0]))
        
        coordinates_degree_err = coordinates_degree_2 - coordinates_degree_1
    
    
        distance = 0.5
        
        degree = self.pid_degrees_straight.feedback(err_degree_convert)
        if (err_distance<0.5) :
            self.coord_index += 1
        
        if self.coord_index == len(coordinates)-6:
            done = True
            self.coordinates = []
        else:
            done = False
        
        
        self.moveCar(distance, degree)
        
        return coordinates_degree_err

    def compute_left(self, coordinates):

        # print("++++++++++++++++++++left++++++++++++++++++")
        if len(self.coordinates) == 0:
            self.coordinates = coordinates
        
        else:
            done = False
        car_state = self.client.getCarState(vehicle_name=self.car_name)

        pos_x = car_state.kinematics_estimated.position.x_val
        
        pos_y = car_state.kinematics_estimated.position.y_val
        self.storePos(pos_x, pos_y)
        target = airsim.Vector3r(coordinates[self.coord_index][0],coordinates[self.coord_index][1],0)

        target_1 = airsim.Vector3r(coordinates[self.coord_index+1][0],coordinates[self.coord_index+1][1],0)

        target_x = target.x_val

        target_1_x = target_1.x_val

        target_y = target.y_val

        target_1_y = target_1.y_val

        err_x = target_x - pos_x

        err_x_1 = target_1_x - pos_x

        err_y = target_y - pos_y

        err_y_1 = target_1_y - pos_y

        err_distance = math.sqrt(err_x ** 2 + err_y ** 2)
       


        
        x = car_state.kinematics_estimated.orientation.x_val
        y = car_state.kinematics_estimated.orientation.y_val
        z = car_state.kinematics_estimated.orientation.z_val
        w = car_state.kinematics_estimated.orientation.w_val
        orientation_z = math.atan2(2*(x*y+z*w),x*x-y*y-z*z+w*w)

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
        
        coordinates_degree_1 = math.degrees(math.atan2(coordinates[self.coord_index+6][1]
        -coordinates[self.coord_index+5][1],coordinates[self.coord_index+6][0]-coordinates[self.coord_index+5][0]))
        
        coordinates_degree_2 = math.degrees(math.atan2(coordinates[self.coord_index+7][1]
        -coordinates[self.coord_index+6][1],coordinates[self.coord_index+7][0]-coordinates[self.coord_index+6][0]))
        
        coordinates_degree_err = coordinates_degree_2 - coordinates_degree_1


        distance = 0.2

        degree = self.pid_degrees_curve.feedback(err_degree_convert)
        if (err_distance<1.5 ) :
            self.coord_index += 1

        if self.coord_index == len(coordinates)-8:
            done = True
            self.coordinates = []
        else:
            done = False    
        
        self.moveCar(distance, degree)
        return coordinates_degree_err 

    def compute_right(self, coordinates):

        # print("++++++++++++++++++++rigth++++++++++++++++++")
        if len(self.coordinates) == 0:
            self.coordinates = coordinates
        
        else:
            done = False
        car_state = self.client.getCarState(vehicle_name=self.car_name)

        pos_x = car_state.kinematics_estimated.position.x_val
        pos_y = car_state.kinematics_estimated.position.y_val

        self.storePos(pos_x, pos_y)
        
        target = airsim.Vector3r(coordinates[self.coord_index][0],coordinates[self.coord_index][1],0)

        target_1 = airsim.Vector3r(coordinates[self.coord_index+1][0],coordinates[self.coord_index+1][1],0)

        target_x = target.x_val

        target_1_x = target_1.x_val

        target_y = target.y_val

        target_1_y = target_1.y_val

        err_x = target_x - pos_x

        err_x_1 = target_1_x - pos_x

        err_y = target_y - pos_y

        err_y_1 = target_1_y - pos_y

        err_distance = math.sqrt(err_x ** 2 + err_y ** 2)
       
        x = car_state.kinematics_estimated.orientation.x_val
        y = car_state.kinematics_estimated.orientation.y_val
        z = car_state.kinematics_estimated.orientation.z_val
        w = car_state.kinematics_estimated.orientation.w_val
        orientation_z = math.atan2(2*(x*y+z*w),x*x-y*y-z*z+w*w)

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
        
        coordinates_degree_1 = math.degrees(math.atan2(coordinates[self.coord_index+6][1]
        -coordinates[self.coord_index+5][1],coordinates[self.coord_index+6][0]-coordinates[self.coord_index+5][0]))
        
        coordinates_degree_2 = math.degrees(math.atan2(coordinates[self.coord_index+7][1]
        -coordinates[self.coord_index+6][1],coordinates[self.coord_index+7][0]-coordinates[self.coord_index+6][0]))
        
        coordinates_degree_err = coordinates_degree_2 - coordinates_degree_1
        
        distance = 0.15

        degree = self.pid_degrees_curve.feedback(err_degree_convert)
        if (err_distance<2 ) :
            self.coord_index += 1

        if self.coord_index == len(coordinates)-6:
            done = True
            self.coordinates = []
        else:
            done = False    

        self.moveCar(distance, degree)
       
        return coordinates_degree_err   

    
            



