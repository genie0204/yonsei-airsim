from .env_modules import vrep
import numpy as np
import time

class Env():
    def __init__(self, port):
        self.clientID = vrep.simxStart('127.0.0.1', port, True, True, 5000, 5)
        self.ego_handles = {}
        self.risky_handles = {}
        self.rear_handles = {}
        self.lateral_handles = {}
        self.left0_handles = {}
        self.left1_handles = {}
        self.opposite_handles = {}
        self.right0_handles = {}
        self.collision_handles = {}
        self.lidar_handles = {}

        # Vehicle lists: ['EGO', 'RISKY', 'REAR', 'LATERAL', 'LEFT0', 'LEFT1', 'OPPOSITE', 'RIGHT0']
        self.motor_list = ['steeringLeft', 'steeringRight', 'motorLeft', 'motorRight']
        self.lidar_list = ['lidar0', 'lidar1', 'lidar2']
        self.collision_list = ['rear', 'near', 'far']
        
        # Set handles
        self.ego_handle = vrep.simxGetObjectHandle(self.clientID, 'EGO', vrep.simx_opmode_blocking)[1]
        self.risky_handle = vrep.simxGetObjectHandle(self.clientID, 'RISKY', vrep.simx_opmode_blocking)[1]
        self.rear_handle = vrep.simxGetObjectHandle(self.clientID, 'REAR', vrep.simx_opmode_blocking)[1]
        self.lateral_handle = vrep.simxGetObjectHandle(self.clientID, 'LATERAL', vrep.simx_opmode_blocking)[1]
        self.left0_handle = vrep.simxGetObjectHandle(self.clientID, 'LEFT0', vrep.simx_opmode_blocking)[1]
        self.left1_handle = vrep.simxGetObjectHandle(self.clientID, 'LEFT1', vrep.simx_opmode_blocking)[1]
        self.opposite_handle = vrep.simxGetObjectHandle(self.clientID, 'OPPOSITE', vrep.simx_opmode_blocking)[1]
        self.right0_handle = vrep.simxGetObjectHandle(self.clientID, 'RIGHT0', vrep.simx_opmode_blocking)[1]
        self.target_handle = vrep.simxGetObjectHandle(self.clientID, 'target', vrep.simx_opmode_blocking)[1]

        for name in self.motor_list:
            self.ego_handles[name] = vrep.simxGetObjectHandle(self.clientID, 'ego_' + name, vrep.simx_opmode_blocking)[1]
            self.risky_handles[name] = vrep.simxGetObjectHandle(self.clientID, 'risky_' + name, vrep.simx_opmode_blocking)[1]
            self.rear_handles[name] = vrep.simxGetObjectHandle(self.clientID, 'rear_' + name, vrep.simx_opmode_blocking)[1]
            self.lateral_handles[name] = vrep.simxGetObjectHandle(self.clientID, 'lateral_' + name, vrep.simx_opmode_blocking)[1]
            self.left0_handles[name] = vrep.simxGetObjectHandle(self.clientID, 'left0_' + name, vrep.simx_opmode_blocking)[1]
            self.left1_handles[name] = vrep.simxGetObjectHandle(self.clientID, 'left1_' + name, vrep.simx_opmode_blocking)[1]
            self.opposite_handles[name] = vrep.simxGetObjectHandle(self.clientID, 'opposite_' + name, vrep.simx_opmode_blocking)[1]
            self.right0_handles[name] = vrep.simxGetObjectHandle(self.clientID, 'right0_' + name, vrep.simx_opmode_blocking)[1]

        for name in self.collision_list:
            self.collision_handles[name] = vrep.simxGetCollisionHandle(self.clientID, name, vrep.simx_opmode_blocking)[1]
        for name in self.lidar_list:
            self.lidar_handles[name] = vrep.simxGetObjectHandle(self.clientID, name, vrep.simx_opmode_blocking)[1]

        vrep.simxSynchronous(self.clientID, True)

        # Set some constants
        self.target = np.array([-3.1252, -16.475])

    def get_lidar(self):
        lidar_array = []
        for name in self.lidar_list:
            buffer = vrep.simxGetVisionSensorDepthBuffer(self.clientID, self.lidar_handles[name], vrep.simx_opmode_blocking)[2]
            lidar_array += buffer
        return lidar_array

    def reset(self):
        vrep.simxSynchronous(self.clientID, True)
        time.sleep(0.5)
        vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_oneshot)
        
        # Random choice of scenario
        self.scenario_num = np.random.randint(3)
        # self.scenario_num = 1

        self.lidar_sequence = []

        # Rule-based control
        if self.scenario_num == 0:
            # Randomizing variables
            self.risky_lane = np.random.randint(2) # 0: 1st lane, 1: 2nd lane
            self.neighbor_lane = np.random.randint(2) # 0: rear, 1: side
            self.opposite_lane = np.random.randint(2) # 0: 1st lane, 1: 2nd lane
            self.right_lane = np.random.randint(2) # 0: 1st lane, 1: 2nd lane

            vrep.simxPauseCommunication(self.clientID, 1)
            vrep.simxSetObjectPosition(self.clientID, self.ego_handle, -1, (-3.1252, 13.525, 0.3677), vrep.simx_opmode_oneshot)
            vrep.simxSetObjectOrientation(self.clientID, self.ego_handle, -1, (np.pi/2.0, 0.0, np.pi/2.0), vrep.simx_opmode_oneshot)
            if self.risky_lane == 0:
                # Set risky position
                vrep.simxSetObjectPosition(self.clientID, self.risky_handle, -1, (46.0, 2.0, 0.3677), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.risky_handle, -1, (0.0, -np.pi/2.0, 0.0), vrep.simx_opmode_oneshot)
                # left0
                vrep.simxSetObjectPosition(self.clientID, self.left0_handle, -1, (16.0, 7.0, 0.3677), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.left0_handle, -1, (0.0, -np.pi/2.0, 0.0), vrep.simx_opmode_oneshot)
                # left1
                vrep.simxSetObjectPosition(self.clientID, self.left1_handle, -1, (23.0, 7.0, 0.3677), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.left1_handle, -1, (0.0, -np.pi/2.0, 0.0), vrep.simx_opmode_oneshot)
            elif self.risky_lane == 1:
                # Set risky position
                vrep.simxSetObjectPosition(self.clientID, self.risky_handle, -1, (46.0, 5.0, 0.3677), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.risky_handle, -1, (0.0, -np.pi/2.0, 0.0), vrep.simx_opmode_oneshot)
                # left0
                vrep.simxSetObjectPosition(self.clientID, self.left0_handle, -1, (16.0, 2.0, 0.3677), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.left0_handle, -1, (0.0, -np.pi/2.0, 0.0), vrep.simx_opmode_oneshot)
                # left1
                vrep.simxSetObjectPosition(self.clientID, self.left1_handle, -1, (23.0, 2.0, 0.3677), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.left1_handle, -1, (0.0, -np.pi/2.0, 0.0), vrep.simx_opmode_oneshot)
            if self.neighbor_lane == 0:
                # Set rear position
                vrep.simxSetObjectPosition(self.clientID, self.rear_handle, -1, (-3.0, 20.0, 0.3677), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.rear_handle, -1, (np.pi/2.0, 0.0, np.pi/2.0), vrep.simx_opmode_oneshot)
                # Locate lateral on parking lot
                vrep.simxSetObjectPosition(self.clientID, self.lateral_handle, -1, (-7.0, 13.0, -7.39), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.lateral_handle, -1, (np.pi/2.0, 0.0, np.pi/2.0), vrep.simx_opmode_oneshot)
            elif self.neighbor_lane == 1:
                # Set lateral position
                vrep.simxSetObjectPosition(self.clientID, self.lateral_handle, -1, (-7.0, 13.0, 0.3677), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.lateral_handle, -1, (np.pi/2.0, 0.0, np.pi/2.0), vrep.simx_opmode_oneshot)
                # Locate rear on parking lot
                vrep.simxSetObjectPosition(self.clientID, self.rear_handle, -1, (-3.0, 20.0, -7.39), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.rear_handle, -1, (np.pi/2.0, 0.0, np.pi/2.0), vrep.simx_opmode_oneshot)
            if self.opposite_lane == 0:    
                vrep.simxSetObjectPosition(self.clientID, self.opposite_handle, -1, (2.0, -15.0, 0.3677), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.opposite_handle, -1, (-np.pi/2.0, 0.0, -np.pi/2.0), vrep.simx_opmode_oneshot)
            elif self.opposite_lane == 1:
                vrep.simxSetObjectPosition(self.clientID, self.opposite_handle, -1, (7.0, -15.0, 0.3677), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.opposite_handle, -1, (-np.pi/2.0, 0.0, -np.pi/2.0), vrep.simx_opmode_oneshot)
            if self.right_lane == 0:
                vrep.simxSetObjectPosition(self.clientID, self.right0_handle, -1, (-15.0, -6.0, 0.3677), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.right0_handle, -1, (0, np.pi/2.0, -np.pi), vrep.simx_opmode_oneshot)
            elif self.right_lane == 1:
                vrep.simxSetObjectPosition(self.clientID, self.right0_handle, -1, (-15.0, -2.0, 0.3677), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.right0_handle, -1, (0, np.pi/2.0, -np.pi), vrep.simx_opmode_oneshot)

            self.set_action('ego', [-1.0, 0.0])
            self.set_action('risky', [-1.0, 0.0])
            self.set_action('rear', [-1.0, 0.0])
            self.set_action('lateral', [-1.0, 0.0])
            self.set_action('opposite', [-1.0, 0.0])
            vrep.simxPauseCommunication(self.clientID, 0)
            vrep.simxSynchronousTrigger(self.clientID)
            
            if self.risky_lane == 0:
                # It takes 73 timesteps for risky to reach the commanded speed.
                self.set_action('risky', [1.0, 0.0])
                for i in range(45):
                    vrep.simxSynchronousTrigger(self.clientID)

                vrep.simxPauseCommunication(self.clientID, 1)
                self.set_action('ego', [-0.4, 0.0])
                self.set_action('opposite', [-0.4, 0.0])
                if self.neighbor_lane == 1:
                    self.set_action('lateral', [-0.4, 0.0])
                vrep.simxPauseCommunication(self.clientID, 0)                    
                for i in range(10):
                    vrep.simxSynchronousTrigger(self.clientID)

                if self.neighbor_lane == 0:
                    self.set_action('rear', [-0.5, 0.0])
                for i in range(18):
                    vrep.simxSynchronousTrigger(self.clientID)

                # Give 15 step chance (collide at 21)
                for i in range(2):
                    vrep.simxSynchronousTrigger(self.clientID)
                for i in range(4):
                    vrep.simxSynchronousTrigger(self.clientID)
                    self.lidar_sequence += self.get_lidar()
                    
            elif self.risky_lane == 1:
                self.set_action('risky', [1.0, 0.0])
                for i in range(58):
                    vrep.simxSynchronousTrigger(self.clientID)
                
                vrep.simxPauseCommunication(self.clientID, 1)
                self.set_action('ego', [-0.4, 0.0])
                self.set_action('opposite', [-0.4, 0.0])
                if self.neighbor_lane == 1:
                    self.set_action('lateral', [-0.4, 0.0])
                vrep.simxPauseCommunication(self.clientID, 0)
                for i in range(10):
                    vrep.simxSynchronousTrigger(self.clientID)
                
                if self.neighbor_lane == 0:
                    self.set_action('rear', [-0.5, 0.0])
                for i in range(5):
                    vrep.simxSynchronousTrigger(self.clientID)

                # Give 15 step chance (collide at 21)
                for i in range(2):
                    vrep.simxSynchronousTrigger(self.clientID)
                for i in range(4):
                    vrep.j(self.clientID)sss
                    self.lidar_sequence += self.get_lidar()
                    
        if self.scenario_num == 1:
            # Randomizing scenario
            self.risky_lane = np.random.randint(2) # 0: 1st lane, 1: 2nd lane
            self.risky_lane = 0
            self.neighbor_lane = np.random.randint(2) # 0: rear, 1: none
            self.opposite_lane = np.random.randint(2) # 0: 1st lane, 1: 2nd lane
            self.right_lane = np.random.randint(2) # 0: 1st lane, 1: 2nd lane

            vrep.simxPauseCommunication(self.clientID, 1)
            vrep.simxSetObjectPosition(self.clientID, self.ego_handle, -1, (-3.1252, 13.525, 0.3677), vrep.simx_opmode_oneshot)
            vrep.simxSetObjectOrientation(self.clientID, self.ego_handle, -1, (np.pi/2.0, 0.0, np.pi/2.0), vrep.simx_opmode_oneshot)
            if self.risky_lane == 0:
                # Set risky position
                vrep.simxSetObjectPosition(self.clientID, self.risky_handle, -1, (-46.0, -2.0, 0.3677), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.risky_handle, -1, (0.0, np.pi/2.0, np.pi), vrep.simx_opmode_oneshot)
                # left0
                vrep.simxSetObjectPosition(self.clientID, self.left0_handle, -1, (-16.0, -7.0, 0.3677), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.left0_handle, -1, (0.0, np.pi/2.0, np.pi), vrep.simx_opmode_oneshot)
                # left1
                vrep.simxSetObjectPosition(self.clientID, self.left1_handle, -1, (-23.0, -7.0, 0.3677), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.left1_handle, -1, (0.0, np.pi/2.0, np.pi), vrep.simx_opmode_oneshot)
            elif self.risky_lane == 1:
                # Set risky position
                vrep.simxSetObjectPosition(self.clientID, self.risky_handle, -1, (-46.0, -5.0, 0.3677), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.risky_handle, -1, (0.0, np.pi/2.0, np.pi), vrep.simx_opmode_oneshot)
                # left0
                vrep.simxSetObjectPosition(self.clientID, self.left0_handle, -1, (-16.0, -2.0, 0.3677), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.left0_handle, -1, (0.0, np.pi/2.0, np.pi), vrep.simx_opmode_oneshot)
                # left1
                vrep.simxSetObjectPosition(self.clientID, self.left1_handle, -1, (-23.0, -2.0, 0.3677), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.left1_handle, -1, (0.0, np.pi/2.0, np.pi), vrep.simx_opmode_oneshot)
            if self.neighbor_lane == 0:
                # Set rear position
                vrep.simxSetObjectPosition(self.clientID, self.rear_handle, -1, (-3.0, 20.0, 0.3677), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.rear_handle, -1, (np.pi/2.0, 0.0, np.pi/2.0), vrep.simx_opmode_oneshot)
                # Locate lateral on parking lot
                vrep.simxSetObjectPosition(self.clientID, self.lateral_handle, -1, (-7.0, 13.0, -7.39), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.lateral_handle, -1, (np.pi/2.0, 0.0, np.pi/2.0), vrep.simx_opmode_oneshot)
            elif self.neighbor_lane == 1:
                # Set lateral position
                vrep.simxSetObjectPosition(self.clientID, self.lateral_handle, -1, (-7.0, 13.0, -7.39), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.lateral_handle, -1, (np.pi/2.0, 0.0, np.pi/2.0), vrep.simx_opmode_oneshot)
                # Locate rear on parking lot
                vrep.simxSetObjectPosition(self.clientID, self.rear_handle, -1, (-3.0, 20.0, -7.39), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.rear_handle, -1, (np.pi/2.0, 0.0, np.pi/2.0), vrep.simx_opmode_oneshot)
            if self.opposite_lane == 0:    
                vrep.simxSetObjectPosition(self.clientID, self.opposite_handle, -1, (2.0, -15.0, 0.3677), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.opposite_handle, -1, (-np.pi/2.0, 0.0, -np.pi/2.0), vrep.simx_opmode_oneshot)
            elif self.opposite_lane == 1:
                vrep.simxSetObjectPosition(self.clientID, self.opposite_handle, -1, (7.0, -15.0, 0.3677), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.opposite_handle, -1, (-np.pi/2.0, 0.0, -np.pi/2.0), vrep.simx_opmode_oneshot)
            if self.right_lane == 0:
                vrep.simxSetObjectPosition(self.clientID, self.right0_handle, -1, (15.0, 6.0, 0.3677), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.right0_handle, -1, (0, -np.pi/2.0, 0), vrep.simx_opmode_oneshot)
            elif self.right_lane == 1:
                vrep.simxSetObjectPosition(self.clientID, self.right0_handle, -1, (15.0, 2.0, 0.3677), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.right0_handle, -1, (0, -np.pi/2.0, 0), vrep.simx_opmode_oneshot)
            
            self.set_action('ego', [-1.0, 0.0])
            self.set_action('risky', [-1.0, 0.0])
            self.set_action('rear', [-1.0, 0.0])
            self.set_action('lateral', [-1.0, 0.0])
            self.set_action('opposite', [-1.0, 0.0])
            vrep.simxPauseCommunication(self.clientID, 0)
            vrep.simxSynchronousTrigger(self.clientID)
            
            if self.risky_lane == 0:
                self.set_action('risky', [1.0, 0.0])
                for i in range(21):
                    vrep.simxSynchronousTrigger(self.clientID)

                vrep.simxPauseCommunication(self.clientID, 1)
                self.set_action('ego', [-0.4, 0.0])
                self.set_action('opposite', [-0.4, 0.0])
                vrep.simxPauseCommunication(self.clientID, 0)    
                for i in range(10):
                    vrep.simxSynchronousTrigger(self.clientID)

                if self.neighbor_lane == 0:
                    self.set_action('rear', [-0.5, 0.0])
                for i in range(32):
                    vrep.simxSynchronousTrigger(self.clientID)

                for i in range(4):
                    vrep.simxSynchronousTrigger(self.clientID)
                    self.lidar_sequence += self.get_lidar()
            
            elif self.risky_lane == 1:
                self.set_action('risky', [1.0, 0.0])
                for i in range(7):
                    vrep.simxSynchronousTrigger(self.clientID)

                vrep.simxPauseCommunication(self.clientID, 1)
                self.set_action('ego', [-0.4, 0.0])
                self.set_action('opposite', [-0.4, 0.0])
                vrep.simxPauseCommunication(self.clientID, 0)    
                for i in range(10):                    
                    vrep.simxSynchronousTrigger(self.clientID)

                if self.neighbor_lane == 0:
                    self.set_action('rear', [-0.5, 0.0])
                for i in range(51):
                    vrep.simxSynchronousTrigger(self.clientID)

                # Give 10 step chance (collide at 21)
                for i in range(4):
                    vrep.simxSynchronousTrigger(self.clientID)
                    self.lidar_sequence += self.get_lidar()

        if self.scenario_num == 2:
            # Randomizing scenario
            self.neighbor_lane = np.random.randint(2) # 0: rear, 1: none
            self.opposite_lane = np.random.randint(2) # 0: 1st lane, 1: 2nd lane
            self.right_lane = np.random.randint(2) # 0: 1st lane, 1: 2nd lane

            vrep.simxPauseCommunication(self.clientID, 1)
            vrep.simxSetObjectPosition(self.clientID, self.ego_handle, -1, (-3.1252, 13.525, 0.3677), vrep.simx_opmode_oneshot)
            vrep.simxSetObjectOrientation(self.clientID, self.ego_handle, -1, (np.pi/2.0, 0.0, np.pi/2.0), vrep.simx_opmode_oneshot)
            # Set risky position
            vrep.simxSetObjectPosition(self.clientID, self.risky_handle, -1, (-30.0, -2.0, 0.3677), vrep.simx_opmode_oneshot)
            vrep.simxSetObjectOrientation(self.clientID, self.risky_handle, -1, (0.0, np.pi/2.0, np.pi), vrep.simx_opmode_oneshot)
            # left0
            vrep.simxSetObjectPosition(self.clientID, self.left0_handle, -1, (-16.0, -7.0, 0.3677), vrep.simx_opmode_oneshot)
            vrep.simxSetObjectOrientation(self.clientID, self.left0_handle, -1, (0.0, np.pi/2.0, np.pi), vrep.simx_opmode_oneshot)
            # left1
            vrep.simxSetObjectPosition(self.clientID, self.left1_handle, -1, (-23.0, -7.0, 0.3677), vrep.simx_opmode_oneshot)
            vrep.simxSetObjectOrientation(self.clientID, self.left1_handle, -1, (0.0, np.pi/2.0, np.pi), vrep.simx_opmode_oneshot)
            if self.neighbor_lane == 0:
                # Set rear position
                vrep.simxSetObjectPosition(self.clientID, self.rear_handle, -1, (-3.0, 20.0, 0.3677), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.rear_handle, -1, (np.pi/2.0, 0.0, np.pi/2.0), vrep.simx_opmode_oneshot)
                # Locate lateral on parking lot
                vrep.simxSetObjectPosition(self.clientID, self.lateral_handle, -1, (-7.0, 13.0, -7.39), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.lateral_handle, -1, (np.pi/2.0, 0.0, np.pi/2.0), vrep.simx_opmode_oneshot)
            elif self.neighbor_lane == 1:
                # Locate lateral on parking lot
                vrep.simxSetObjectPosition(self.clientID, self.lateral_handle, -1, (-7.0, 13.0, -7.39), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.lateral_handle, -1, (np.pi/2.0, 0.0, np.pi/2.0), vrep.simx_opmode_oneshot)
                # Locate rear on parking lot
                vrep.simxSetObjectPosition(self.clientID, self.rear_handle, -1, (-3.0, 20.0, -7.39), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.rear_handle, -1, (np.pi/2.0, 0.0, np.pi/2.0), vrep.simx_opmode_oneshot)
            if self.opposite_lane == 0:
                vrep.simxSetObjectPosition(self.clientID, self.opposite_handle, -1, (2.0, -15.0, 0.3677), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.opposite_handle, -1, (-np.pi/2.0, 0.0, -np.pi/2.0), vrep.simx_opmode_oneshot)
            elif self.opposite_lane == 1:
                vrep.simxSetObjectPosition(self.clientID, self.opposite_handle, -1, (7.0, -15.0, 0.3677), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.opposite_handle, -1, (-np.pi/2.0, 0.0, -np.pi/2.0), vrep.simx_opmode_oneshot)
            if self.right_lane == 0:
                vrep.simxSetObjectPosition(self.clientID, self.right0_handle, -1, (15.0, 6.0, 0.3677), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.right0_handle, -1, (0, -np.pi/2.0, 0), vrep.simx_opmode_oneshot)
            elif self.right_lane == 1:
                vrep.simxSetObjectPosition(self.clientID, self.right0_handle, -1, (15.0, 2.0, 0.3677), vrep.simx_opmode_oneshot)
                vrep.simxSetObjectOrientation(self.clientID, self.right0_handle, -1, (0, -np.pi/2.0, 0), vrep.simx_opmode_oneshot)
            
            self.set_action('ego', [-1.0, 0.0])
            self.set_action('risky', [-1.0, 0.0])
            self.set_action('rear', [-1.0, 0.0])
            self.set_action('lateral', [-1.0, 0.0])
            self.set_action('opposite', [-1.0, 0.0])
            vrep.simxPauseCommunication(self.clientID, 0)
            vrep.simxSynchronousTrigger(self.clientID)
            
            self.set_action('risky', [0.5, 0.0])
            for i in range(27):
                vrep.simxSynchronousTrigger(self.clientID)
            
            self.set_action('ego', [-0.4, 0.0])
            for i in range(30):
                vrep.simxSynchronousTrigger(self.clientID)

            self.set_action('risky', [0.5, 0.6])
            # for i in range(4):
            #     vrep.simxSynchronousTrigger(self.clientID)
            for i in range(4):
                vrep.simxSynchronousTrigger(self.clientID)
                self.lidar_sequence += self.get_lidar()
            # 35 steps of turning and then straight

        # target_position = vrep.simxGetObjectPosition(self.clientID, self.target_handle ,self.ego_handle, vrep.simx_opmode_blocking)[1]
        # target_x = target_position[1]
        # target_y = target_position[2]
        # dist_target = (target_x**2 + target_y**2)**0.5
        # target_state = [target_x/30.0, target_y/30.0]
        state = self.lidar_sequence + [-0.4, 0.0]# + target_state
        velocity = vrep.simxGetObjectVelocity(self.clientID, self.ego_handle, vrep.simx_opmode_blocking)[1]
        speed = (velocity[0]**2 + velocity[1]**2 + velocity[2]**2)**0.5
        # self.prev_distance = dist_target
        # self.prev_velocity = velocity
        self.prev_speed = speed
        return state

    def step(self, action, timestep):
        done = False

        vrep.simxPauseCommunication(self.clientID, 1)
        self.set_action('ego', action)
        if self.scenario_num == 2 and timestep == 31:
            self.set_action('risky', [0.5, 0.0])
        vrep.simxPauseCommunication(self.clientID, 0)
        vrep.simxSynchronousTrigger(self.clientID)

        self.lidar_sequence = self.lidar_sequence[96:] + self.get_lidar()
        
        # target_position = vrep.simxGetObjectPosition(self.clientID, self.target_handle ,self.ego_handle, vrep.simx_opmode_blocking)[1]
        # target_x = target_position[1]
        # target_y = target_position[2]
        # dist_target = (target_x**2 + target_y**2)**0.5
        # target_state = [target_x/30.0, target_y/30.0]
        state1 = self.lidar_sequence + action.tolist()# + target_state

        velocity = vrep.simxGetObjectVelocity(self.clientID, self.ego_handle, vrep.simx_opmode_blocking)[1]
        speed = (velocity[0]**2 + velocity[1]**2 + velocity[2]**2)**0.5
        
        # Get collision area
        collision = {}
        for name in self.collision_list:
            collision[name] = vrep.simxReadCollision(self.clientID, self.collision_handles[name], vrep.simx_opmode_blocking)[1]
        
        # Get alpha
        if collision['rear'] or collision['far'] or collision['near']:
            if collision['rear']:
                alpha = 1.991
            if collision['far']:
                alpha = 2.038
            if collision['near']:
                alpha = 2.698
            
            vrep.simxSynchronousTrigger(self.clientID)
            vrep.simxSynchronousTrigger(self.clientID)
            velocity1 = vrep.simxGetObjectVelocity(self.clientID, self.ego_handle, vrep.simx_opmode_blocking)[1]
            speed1 = (velocity1[0]**2 + velocity1[1]**2 + velocity1[2]**2)**0.5
            
            deltaV = speed1 - self.prev_speed
            collision_reward = -np.exp(2.011*10**(-7)*0.5*836.0*deltaV**2 + alpha)
            done = True
        elif timestep > 50:
            done = True
            collision_reward = 0.0
        else:
            collision_reward = 0.0
        
        # target_reward = self.prev_distance - dist_target
        reward = collision_reward# + target_reward

        # print('collision:',collision_reward, 'target:',target_reward)
        
        if done == True:
            vrep.simxPauseCommunication(self.clientID, 1)
            self.set_action('ego', [-1.0, 0.0])
            self.set_action('risky', [-1.0, 0.0])
            self.set_action('rear', [-1.0, 0.0])
            self.set_action('opposite', [-1.0, 0.0])
            vrep.simxPauseCommunication(self.clientID, 0)
            vrep.simxSynchronousTrigger(self.clientID)
            vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_blocking)

        # Update prev_distance, velocity
        # self.prev_velocity = velocity
        self.prev_speed = speed
        # self.prev_distance = dist_target
        return state1, reward, done

    # Send action signal to simulator
    def set_action(self, type, action):
        d = 0.755
        l = 2.5772
        desiredSpeed, desiredSteeringAngle = action
        desiredSpeed = 25. * desiredSpeed + 25
        desiredSteeringAngle = 0.785 * desiredSteeringAngle

        if desiredSteeringAngle != 0.:
            steeringAngleLeft = np.arctan(l / (-d + l / np.tan(desiredSteeringAngle)))
            steeringAngleRight = np.arctan(l / (d + l / np.tan(desiredSteeringAngle)))
        else:
            steeringAngleLeft = 0.
            steeringAngleRight = 0.
        
        if type == 'ego':
            vrep.simxSetJointTargetVelocity(self.clientID, self.ego_handles['motorLeft'], desiredSpeed, vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetVelocity(self.clientID, self.ego_handles['motorRight'], desiredSpeed, vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetPosition(self.clientID, self.ego_handles['steeringLeft'], steeringAngleLeft, vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetPosition(self.clientID, self.ego_handles['steeringRight'], steeringAngleRight, vrep.simx_opmode_oneshot)
        elif type == 'risky':
            vrep.simxSetJointTargetVelocity(self.clientID, self.risky_handles['motorLeft'], desiredSpeed, vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetVelocity(self.clientID, self.risky_handles['motorRight'], desiredSpeed, vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetPosition(self.clientID, self.risky_handles['steeringLeft'], steeringAngleLeft, vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetPosition(self.clientID, self.risky_handles['steeringRight'], steeringAngleRight, vrep.simx_opmode_oneshot)
        elif type == 'rear':
            vrep.simxSetJointTargetVelocity(self.clientID, self.rear_handles['motorLeft'], desiredSpeed, vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetVelocity(self.clientID, self.rear_handles['motorRight'], desiredSpeed, vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetPosition(self.clientID, self.rear_handles['steeringLeft'], steeringAngleLeft, vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetPosition(self.clientID, self.rear_handles['steeringRight'], steeringAngleRight, vrep.simx_opmode_oneshot)
        elif type == 'opposite':
            vrep.simxSetJointTargetVelocity(self.clientID, self.opposite_handles['motorLeft'], desiredSpeed, vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetVelocity(self.clientID, self.opposite_handles['motorRight'], desiredSpeed, vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetPosition(self.clientID, self.opposite_handles['steeringLeft'], steeringAngleLeft, vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetPosition(self.clientID, self.opposite_handles['steeringRight'], steeringAngleRight, vrep.simx_opmode_oneshot)
        elif type == 'lateral':
            vrep.simxSetJointTargetVelocity(self.clientID, self.lateral_handles['motorLeft'], desiredSpeed, vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetVelocity(self.clientID, self.lateral_handles['motorRight'], desiredSpeed, vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetPosition(self.clientID, self.lateral_handles['steeringLeft'], steeringAngleLeft, vrep.simx_opmode_oneshot)
            vrep.simxSetJointTargetPosition(self.clientID, self.lateral_handles['steeringRight'], steeringAngleRight, vrep.simx_opmode_oneshot)


    def close(self):
        for name in self.collision_list:
            vrep.simxReadCollision(self.clientID, self.collision_handles[name], vrep.simx_opmode_discontinue)
        vrep.simxGetObjectOrientation(self.clientID, self.ego_handle, self.risky_handle, vrep.simx_opmode_discontinue)
        vrep.simxGetObjectPosition(self.clientID, self.ego_handle, -1, vrep.simx_opmode_discontinue)
        for name in self.lidar_list:
            vrep.simxGetVisionSensorDepthBuffer(self.clientID, self.lidar_handles[name], vrep.simx_opmode_discontinue)
        vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_blocking)
        time.sleep(0.5)        