#-*- coding: utf-8 -*-
from __future__ import print_function, absolute_import, division
import time
import sys

import math
import setup_path 
import airsim
import numpy as np



from PID_controller_test import PID, CarController
from astar import Astar


if __name__=='__main__':

    #connect to airsim
    client = airsim.CarClient()
    client.confirmConnection()

    car_controller = CarController(client, "Car1")

    astar = Astar(json_file="./utils/airsim_nh.json")

    done = False
    coordinates_degree_err = 0
    d=0
    while True:
        start_point, start_direction, coordinates = astar.compute(path_continue=done)
        
        if done == False:
            # 최초 상태일때 차량 이동 그 이후에는 직전 end_point에서 바로 시작

            car_controller.setPosition(start_point, start_direction)
            time.sleep(3)        
        
        #pid
        while True :

        
            if coordinates_degree_err %90 ==0 :    

               

                time.sleep(0.05)

                coordinates_degree_err = car_controller.compute_straight(coordinates)
            

            
            elif coordinates_degree_err < 0 :

                for i in range (130) :

                    time.sleep(0.05)

                    coordinates_degree_err = car_controller.compute_left(coordinates)
                
            else :

                for i in range (130) :
                        
                    time.sleep(0.05)

                    coordinates_degree_err = car_controller.compute_right(coordinates)
                    
        car_controller.coord_index = 1


        


