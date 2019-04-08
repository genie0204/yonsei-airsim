#-*- coding: utf-8 -*-
from __future__ import print_function, absolute_import, division
import time
import sys

import math
# import setup_path 
import airsim
import numpy as np
# import astar 


from PID_controller import PID, CarController
from utils.astar import Astar

if __name__=='__main__':

    #connect to airsim
    client = airsim.CarClient("192.168.0.51")
    client.confirmConnection()

    car_controller = CarController(client, "Car1")

    astar = Astar(json_file="./utils/airsim_nh.json")

    done = False
    while True:
        start_point, start_direction, coordinates = astar.compute(path_continue=done)

        if done == False:
            # 최초 상태일때 차량 이동 그 이후에는 직전 end_point에서 바로 시작
            car_controller.setPosition(start_point, start_direction)
            time.sleep(1)
        
        # exit(0)
        
        #pid
        while True :
            time.sleep(0.05)
            done, center_distance = car_controller.compute(coordinates)
            # print("Center Distance", center_distance)
            if center_distance > 4:
                # 경로 이탈
                if car_controller.stopCar():
                    done = False
                    time.sleep(3)
                    break

            if done:
                print("Done")
                break

        car_controller.coord_index = 1


        


