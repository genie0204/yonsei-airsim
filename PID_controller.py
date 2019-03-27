from __future__ import print_function, absolute_import, division

import time

import sys

import math
import setup_path 
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







if __name__=='__main__':

   
    #connect to airsim
    client = airsim.CarClient()

    client.confirmConnection()

    client.enableApiControl(True, "Car1")

    car_controls = airsim.CarControls()
    client.setCarControls(car_controls)

    
    #pid
    for t in range(1,200) :
        time.sleep(0.05)
        car_state = client.getCarState()
        pos_x = car_state.kinematics_estimated.position.x_val
        pos_y = car_state.kinematics_estimated.position.y_val
        
        target = airsim.Vector3r(-8,-80,0)    
        target_x = target.x_val
        target_y = target.y_val

        err_x = target_x - pos_x
        err_y = target_y - pos_y
    
        err_distance = math.sqrt(err_x ** 2 + err_y ** 2)
        print (err_distance)

        #orientation_x = car_state.kinematics_estimated.orientation.x_val
        #orientation_y = car_state.kinematics_estimated.orientation.y_val
        #orientation_abs = math.sqrt(orientation_x**2 + orientation_y**2)

        e1_x = err_x/err_distance
        e1_y = err_y/err_distance
        #e2_x = orientation_x/orientation_abs
        #e2_y = orientation_y/orientation_abs        
        orientation_z = (90-math.degrees(math.acos(car_state.kinematics_estimated.orientation.z_val)))*2
        
        target_degree = math.degrees(math.atan2(e1_y,e1_x))
        err_degree = target_degree - orientation_z
        

        #print(pos_x,pos_y)
        print(target_degree)
        print(orientation_z)
        print(err_degree)
        
    
        #err_degree = math.acos(1-)
        
        #err_rotation = target_y - pos_y
        #print (err_distance,err_y)
        
        pid_distance = PID(dt=0.05, Kp=1.5, Ki=0.1, Kd=0.1)
        pid_degrees = PID(dt=0.05, Kp=10, Ki=0.1, Kd=0.1)
        
        distance = pid_distance.feedback(err_distance)
        degree = pid_degrees.feedback(err_degree)
        print (degree)
        
        car_controls = airsim.CarControls()
        car_controls.throttle = distance/100
        car_controls.steering = degree/360

        
        
        client.setCarControls(car_controls)
        car_state = client.getCarState()
        pos_x = car_state.kinematics_estimated.position.x_val
        pos_y = car_state.kinematics_estimated.position.y_val

        print (pos_x, pos_y)

    
