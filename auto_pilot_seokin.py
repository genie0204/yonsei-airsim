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
    coordinates_degree = 0
    d=0
    while True:
        start_point, start_direction, coordinates = astar.compute(path_continue=done)
        #a = len(coordinates)
        #coordinates_sparse = [ coordinates[i] for i in range(a) if i % 3 ==0]

        if done == False:
            # 최초 상태일때 차량 이동 그 이후에는 직전 end_point에서 바로 시작

            car_controller.setPosition(start_point, start_direction)
            time.sleep(3)
        
        
        
        
        #pid
        while True :

            

        

            

            

            if coordinates_degree %90 ==0 :    

               

                time.sleep(0.05)

                _, d, coordinates_degree,err_degree = car_controller.compute_1(coordinates)


        

    

             

            # print("Center Distance", center_distance)

            

            else :

                for i in range (100) :

                    time.sleep(0.05)

                    _, d, coordinates_degree,err_degree = car_controller.compute_2(coordinates)

                


                    


        car_controller.coord_index = 1


        
