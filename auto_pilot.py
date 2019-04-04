# import setup_path
import airsim
import time
import math

from PID_controller import PID
from utils.astar import Astar

if __name__=='__main__':

    #connect to airsim
    client = airsim.CarClient(ip="192.168.0.51")

    client.confirmConnection()

    client.enableApiControl(True, "Car1")

    car_controls = airsim.CarControls()
    client.setCarControls(car_controls)

    astar = Astar(json_file="./utils/airsim_nh.json")

    while True:
        start_point, start_direction, coordinates = astar.compute()

        car_controls.position = airsim.Vector3r(start_point[0],start_point[1],-1)
        car_controls.heading = airsim.utils.to_quaternion(0,0,start_direction)
        car_controls.pose = airsim.Pose(car_controls.position,car_controls.heading)

        client.simSetVehiclePose(car_controls.pose, True,vehicle_name="Car1")
        exit(0)
        time.sleep(5)

    exit(0)

    cur_index = 1
    time.sleep(3)
    #pid
    while True :
        time.sleep(0.05)
        car_state = client.getCarState()
        pos_x = car_state.kinematics_estimated.position.x_val
        pos_y = car_state.kinematics_estimated.position.y_val
        # print()
        # print(cur_index, pos_x, pos_y, coordinates[cur_index][0],coordinates[cur_index][1])
        target = airsim.Vector3r(coordinates[cur_index][0],coordinates[cur_index][1],0)    
        target_x = target.x_val
        target_y = target.y_val

        err_x = target_x - pos_x
        err_y = target_y - pos_y
        # print(err_x, err_y)
    
        err_distance = math.sqrt(err_x ** 2 + err_y ** 2)
        # print (err_distance)
        if (err_distance < 2):
            cur_index += 1
        elif (err_distance > 3 and cur_index > 0):
            cur_index -= 1

        #orientation_x = car_state.kinematics_estimated.orientation.x_val
        #orientation_y = car_state.kinematics_estimated.orientation.y_val
        #orientation_abs = math.sqrt(orientation_x**2 + orientation_y**2)

        e1_x = err_x/err_distance
        e1_y = err_y/err_distance

        e2_y = start_direction_y
        e2_x = start_direction_x

        print(e1_x, e1_y, e2_y, e2_x)
        #e2_x = orientation_x/orientation_abs
        #e2_y = orientation_y/orientation_abs        
        orientation_z = (90-math.degrees(math.acos(car_state.kinematics_estimated.orientation.z_val)))*2
        
        target_degree = math.degrees(math.atan2(e1_y,e1_x-e2_x))
        err_degree = target_degree - orientation_z
        

        #print(pos_x,pos_y)
        print(target_degree)
        # print(orientation_z)
        # print(err_degree)
        
    
        #err_degree = math.acos(1-)
        
        #err_rotation = target_y - pos_y
        #print (err_distance,err_y)
        
        pid_distance = PID(dt=0.05, Kp=1.5, Ki=0.1, Kd=0.1)
        pid_degrees = PID(dt=0.05, Kp=10, Ki=0.1, Kd=0.1)
        
        distance = pid_distance.feedback(err_distance)
        degree = pid_degrees.feedback(err_degree)
        # print(degree)
        
        car_controls = airsim.CarControls()
        car_controls.throttle = distance/10
        car_controls.steering = degree/360
        # print(car_controls)
        client.setCarControls(car_controls)
        car_state = client.getCarState()
        pos_x = car_state.kinematics_estimated.position.x_val
        pos_y = car_state.kinematics_estimated.position.y_val
        
        # print (pos_x, pos_y)

