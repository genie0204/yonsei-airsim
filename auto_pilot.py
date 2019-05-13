#-*- coding: utf-8 -*-
from __future__ import print_function, absolute_import, division
import time
import sys

import math
# import setup_path 
import airsim
import numpy as np

import matplotlib.pyplot as plt

from PID_controller import PID, CarController
from utils.astar import Astar

class AutoPilot():
    def __init__(self, ip=""):
        #connect to airsim
        self.client = airsim.CarClient(ip)
        self.client.confirmConnection()

        self.client2 = airsim.CarClient(ip)
        self.client2.confirmConnection()

        self.client_image = airsim.CarClient(ip)
        self.client_image.confirmConnection()


        self.car_controller = CarController(self.client, "Car1")
        self.car_controller2 = CarController(self.client2, "Car2")

        self.astar = Astar(json_file="./utils/airsim_nh.json")

        self.loop_limit = 130

    def reset(self):
        start_point, start_direction, self.coordinates, r_start_point, r_start_direction, self.r_coordinates = self.astar.compute()
    
        self.car_controller.setPosition(start_point, start_direction)
        self.car_controller2.setPosition(r_start_point, r_start_direction)
        self.car_controller.stopCar()
        self.car_controller2.stopCar()
        time.sleep(1.5)        

        self.coordinates_degree_err = 0
        self.coordinates_degree_err2 = 0

        self.l = 0
        self.l2 = 0

        self.r = 0
        self.r2 = 0

        # return states

    def getCarDistance(self):
        x1, y1 = self.car_controller.getPosition(self.coordinates)
        x2, y2 = self.car_controller2.getPosition(self.r_coordinates)

        return math.sqrt((x1-x2)**2+(y1-y2)**2)

    def getImage(self,car_name):

        if car_name == "Car1" :
            responses = self.client_image.simGetImages([
            airsim.ImageRequest("MyCam1", airsim.ImageType.Scene),  
            airsim.ImageRequest("MyCam2", airsim.ImageType.Scene), 
            airsim.ImageRequest("MyCam3", airsim.ImageType.Scene), 
            airsim.ImageRequest("MyCam4", airsim.ImageType.Scene)])

        if car_name == "Car2" :

            responses = self.client_image.simGetImages([
    
            airsim.ImageRequest("MyCam5", airsim.ImageType.Scene),
            airsim.ImageRequest("MyCam6", airsim.ImageType.Scene),
            airsim.ImageRequest("MyCam7", airsim.ImageType.Scene),
            airsim.ImageRequest("MyCam8", airsim.ImageType.Scene)]) 
            
        return responses

    def getCollisioninfo(self,car_name):

        if car_name=="Car1":
            collision_info = client.simGetCollisionInfo()
            if collision_info.has_collided:
                print("Collision at pos %s, normal %s, impact pt %s, penetration %f, name %s, obj id %d" % (
                    pprint.pformat(collision_info.position), 
                    pprint.pformat(collision_info.normal), 
                    pprint.pformat(collision_info.impact_point), 
                    collision_info.penetration_depth, collision_info.object_name, collision_info.object_id))
                break
            time.sleep(0.1)

        if car_name=="Car2":
            collision_info = client.simGetCollisionInfo()
            if collision_info.has_collided:
                print("Collision at pos %s, normal %s, impact pt %s, penetration %f, name %s, obj id %d" % (
                    pprint.pformat(collision_info.position), 
                    pprint.pformat(collision_info.normal), 
                    pprint.pformat(collision_info.impact_point), 
                    collision_info.penetration_depth, collision_info.object_name, collision_info.object_id))
                break
            time.sleep(0.1)    

        return collision_info
    

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



    def getDistanceFromCenterLine(self,car_name):
                    
        if car_name=="Car1":
            pos_x = car_state.kinematics_estimated.position.x_val
            pos_y = car_state.kinematics_estimated.position.y_val
            self.storePos(pos_x, pos_y)
            target = airsim.Vector3r(coordinates[self.coord_index][0],coordinates[self.coord_index][1],0)
            target_x = target.x_val
            target_y = target.y_val
            
            return math.sqrt((pos_x-target_x)**2+(pos_y-target_y)**2)

     
        if car_name=="Car2":

            pos_x = car_state.kinematics_estimated.position.x_val
            pos_y = car_state.kinematics_estimated.position.y_val
            self.storePos(pos_x, pos_y)
            target = airsim.Vector3r(coordinates[self.coord_index][0],coordinates[self.coord_index][1],0)
            target_x = target.x_val
            target_y = target.y_val

            return math.sqrt((pos_x-target_x)**2+(pos_y-target_y)**2)

    
    def getSpeed(self,car_name):
        if car_name == "Car1": car_state = self.client.getCarState()
        if car_name == "Car2": car_state = self.client2.getCarState()

        return car_state.speed

    def autoDrive(self):
        try:
            time.sleep(0.04)
            if self.coordinates_degree_err %90 == 0 and (self.l == 0 and self.r == 0):
                self.coordinates_degree_err = self.car_controller.compute_straight(self.coordinates)
        
            elif self.coordinates_degree_err < 0 or self.l > 0:
                self.l += 1
                self.coordinates_degree_err = self.car_controller.compute_left(self.coordinates)
                if self.l == self.loop_limit:
                    self.l = 0
                
            elif self.coordinates_degree_err > 0 or self.r > 0:
                self.r += 1
                self.coordinates_degree_err = self.car_controller.compute_right(self.coordinates)
                if self.r == self.loop_limit:
                    self.r = 0

            if self.coordinates_degree_err2 %90 ==0 and (self.l2 ==0 and self.r2 ==0) :
                self.coordinates_degree_err2 = self.car_controller2.compute_straight(self.r_coordinates)
        
            elif self.coordinates_degree_err2 < 0 or self.l2 > 0:
                self.l2 += 1
                self.coordinates_degree_err2 = self.car_controller2.compute_left(self.r_coordinates)
                if self.l2 == self.loop_limit:
                    self.l2 = 0
                
            elif self.coordinates_degree_err2 > 0 or self.r2 > 0:
                self.r2 += 1
                self.coordinates_degree_err2 = self.car_controller2.compute_right(self.r_coordinates)
                if self.r2 == self.loop_limit:
                    self.r2 = 0

        except Exception as ex:
            self.done = True
            self.car_controller.stopCar()
            self.car_controller2.stopCar()

            self.car_controller.coord_index = 1
            self.car_controller2.coord_index = 1
    
    def drive(self,car_name, action):
        car_controls = airsim.CarControls()
        
        car_controls.steering = action[0]
        car_controls.throttle = action[1]
        car_controls.brake = action[2]
        if car_name == "Car1" :
            self.client.setCarControls(car_controls)
            

        if car_name == "Car2" :
            self.client2.setCarControls(car_controls)
            


    def trackCoords(self):
        self.car_controller.keepTrack()
        self.car_controller2.keepTrack()

    def step(self,action,car_name):
        self.done = False

        # Distance btw cars
        d = self.getCarDistance()

        #-----------------#
        # Autopilot
        #-----------------#
        if d > 15:
            self.autoDrive()
            

        #-----------------#

        #-----------------#
        # Reinforcement Learning
        #-----------------#
        else:
            self.keepTrackCoords() # RL 상황 종료 후 지속적인 auto pilot을 위해 사용
            self.drive(car_name,action)
            
            images = self.getImage(car_name)
            speed = self.getSpeed(car_name)
            obs = images.append(speed)

            reward = self.getReward(obs)

            

            if done:
             client.reset()
             car_control = interpret_action(1)
             client.setCarControls(car_control)
             time.sleep(1)
             current_step +=1
        
        return None, self.done

if __name__=='__main__':

    auto_pilot = AutoPilot(ip="192.168.0.51")

    episode_cnt = 0
    while True:
        
        auto_pilot.reset()
        while True :
            states, done = auto_pilot.step()
            if done:
                break
                
        print("Episode ", episode_cnt, " Finished")
        episode_cnt += 1


        





        


