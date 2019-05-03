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

    def drive(self):
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

    def trackCoords(self):
        self.car_controller.keepTrack()
        self.car_controller2.keepTrack()

    def step(self):
        self.done = False

        # Distance btw cars
        d = self.getCarDistance()

        #-----------------#
        # Autopilot
        #-----------------#
        if d > 15:
            self.drive()

        #-----------------#

        #-----------------#
        # Reinforcement Learning
        #-----------------#
        else:
            self.keepTrackCoords() # RL 상황 종료 후 지속적인 auto pilot을 위해 사용
            print("RL Init()")
            
        
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


        


