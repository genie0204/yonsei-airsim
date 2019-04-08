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

        self.pid_distance_staright= PID(dt=0.05, Kp=0.2, Ki=0.001, Kd=0)
        self.pid_degrees_straight = PID(dt=0.05, Kp=1, Ki=0.001, Kd=0)
        self.pid_distance_curve = PID(dt=0.05, Kp=0.1, Ki=0.001, Kd=0)
        self.pid_degrees_curve = PID(dt=0.05, Kp=1, Ki=0.001, Kd=0)

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

            if self.client.getCarState().speed < 0.001:
                break

        return True

    def compute(self, coordinates):
        if len(self.coordinates) == 0:
            self.coordinates = coordinates
        
        car_state = self.client.getCarState()

        pos_x = car_state.kinematics_estimated.position.x_val
        pos_y = car_state.kinematics_estimated.position.y_val
        target = airsim.Vector3r(coordinates[self.coord_index][0],coordinates[self.coord_index][1],0)    
        target_x = target.x_val
        target_y = target.y_val
        # print(target)

        err_x = target_x - pos_x
        err_y = target_y - pos_y
    
        err_distance = math.sqrt(err_x ** 2 + err_y ** 2)
        # print (err_distance)
        if (err_distance < 2):
            self.coord_index += 1

        if self.coord_index == len(coordinates):
            done = True
            self.coordinates = []
        else:
            done = False

        orientation_z = car_state.kinematics_estimated.orientation.z_val * math.pi
        target_degree = math.atan2(err_y,err_x)
        err_degree = target_degree - orientation_z

        
        while True:
            if err_degree > math.pi:
                err_degree -= math.pi
            elif err_degree < -math.pi:
                err_degree += math.pi
            else:
                break

        # 짜잘한 움직임을 제한하기 위해 0도 근처 값 제한
        if abs(err_degree) < math.pi / 24:
            err_degree = 0
    
        try:
            # print(math.degrees(err_degree), math.degrees(target_degree), math.degrees(orientation_z))
            coordinates_degree = math.degrees(math.atan2(coordinates[self.coord_index+10][1]-coordinates[self.coord_index+9][1],
                                                coordinates[self.coord_index+10][0]-coordinates[self.coord_index+9][0]))
        except:
            coordinates_degree = 0

        
        if coordinates_degree % 90 ==0 :
            distance = self.pid_distance_staright.feedback(err_distance)
            degree = self.pid_degrees_straight.feedback(err_degree)
        
        else :            
            distance = self.pid_distance_curve.feedback(err_distance)
            degree = self.pid_degrees_curve.feedback(err_degree)

        self.moveCar(distance, degree)
        d = self.getDistanceFromStraightLine((pos_x, pos_y))

        print(d, err_distance, math.degrees(err_degree), done)
        return done, d

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

    
