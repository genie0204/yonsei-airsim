#-*- coding: utf-8 -*-
from __future__ import print_function, absolute_import, division
import time
import sys

import math
import setup_path 
import airsim
import numpy as np
import astar 


from PID_controller import PID
from astar import Astar

if __name__=='__main__':

    #connect to airsim
    client = airsim.CarClient()

    client.confirmConnection()

    client.enableApiControl(True, "Car1")

    car_controls = airsim.CarControls()
    client.setCarControls(car_controls)

    astar = Astar(json_file="./utils/airsim_nh_binary.json")
    start_point, start_direction, coordinates = astar.compute()
    a = len(coordinates)
    print (coordinates)

    car_controls.position = airsim.Vector3r(start_point[0],start_point[1],0)
    car_controls.heading = airsim.utils.to_quaternion(0,0,start_direction)
    car_controls.pose = airsim.Pose(car_controls.position,car_controls.heading)

    client.simSetVehiclePose(car_controls.pose, True,vehicle_name="Car1")
    
    

    cur_index = 1
    time.sleep(3)
    #pid
    while True :
        time.sleep(0.05)
        car_state = client.getCarState()
        pos_x = car_state.kinematics_estimated.position.x_val
        pos_y = car_state.kinematics_estimated.position.y_val
        target = airsim.Vector3r(coordinates[cur_index][0],coordinates[cur_index][1],0)    
        target_x = target.x_val
        target_y = target.y_val
        print(target)

        err_x = target_x - pos_x
        err_y = target_y - pos_y
    
        err_distance = math.sqrt(err_x ** 2 + err_y ** 2)
        print (err_distance)
        if (err_distance < 2):
            cur_index += 1
        elif (err_distance > 20) :
            print("reset")
            client.reset()
            time.sleep(1)
            quit(0)
               
        orientation_z = car_state.kinematics_estimated.orientation.z_val * math.pi
        target_degree = math.atan2(err_y,err_x)
        err_degree = target_degree - orientation_z
        
        print(err_degree)
        
        pid_distance_staright= PID(dt=0.05, Kp=0.2, Ki=0.001, Kd=0)
        pid_degrees_straight = PID(dt=0.05, Kp=1, Ki=0.001, Kd=0)
        pid_distance_curve = PID(dt=0.05, Kp=0.1, Ki=0.001, Kd=0)
        pid_degrees_curve = PID(dt=0.05, Kp=1, Ki=0.001, Kd=0)
        
        coordinates_degree = math.degrees(math.atan2(coordinates[cur_index+10][1]-coordinates[cur_index+9][1],
        coordinates[cur_index+10][0]-coordinates[cur_index+9][0]))
        print("coordinates_degree",coordinates_degree)
        if coordinates_degree % 90 ==0 :
            distance = pid_distance_staright.feedback(err_distance)
            degree = pid_degrees_straight.feedback(err_degree)
        
        #이부분은 커브길에서의 속도감속을 위해 만든 부분인데 코딩이 미숙해서 Class선언을 못해 중복된부분 양해부탁드립니다..
        
        else :
            print ('change\n\n\n\n\n\n\n\n\n')
            for i in range(100) :
                time.sleep(0.05)
                car_state = client.getCarState()
                pos_x = car_state.kinematics_estimated.position.x_val
                pos_y = car_state.kinematics_estimated.position.y_val
        
                target = airsim.Vector3r(coordinates[cur_index][0],coordinates[cur_index][1],0)    
                target_x = target.x_val
                target_y = target.y_val
                print(target)

                err_x = target_x - pos_x
                err_y = target_y - pos_y
        
                err_distance = math.sqrt(err_x ** 2 + err_y ** 2)
                print (err_distance)
                if (err_distance < 2):
                    cur_index += 1
                elif (err_distance > 20) :
                    print("reset")
                    client.reset()
                    time.sleep(1)
                    quit(0)
         
                orientation_z = car_state.kinematics_estimated.orientation.z_val * math.pi
                target_degree = math.atan2(err_y,err_x)
                err_degree = target_degree - orientation_z


                print(err_degree)
            
                distance = pid_distance_curve.feedback(err_distance)
                degree = pid_degrees_curve.feedback(err_degree)
                car_controls = airsim.CarControls()
                car_controls.throttle = distance
                car_controls.steering = degree
       
                client.setCarControls(car_controls)
                car_state = client.getCarState()
                pos_x = car_state.kinematics_estimated.position.x_val
                pos_y = car_state.kinematics_estimated.position.y_val
        
                print ("car_pos", pos_x, pos_y)            
        
        
        car_controls = airsim.CarControls()
        car_controls.throttle = distance
        car_controls.steering = degree
       
        client.setCarControls(car_controls)
        car_state = client.getCarState()
        pos_x = car_state.kinematics_estimated.position.x_val
        pos_y = car_state.kinematics_estimated.position.y_val
        
        print ("car_pos", pos_x, pos_y)

