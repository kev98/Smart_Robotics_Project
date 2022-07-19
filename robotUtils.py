import sim as vrep
import math
from random import random
import sys


# start a new connection with the CoppeliaSim environment
def start_connection(connection_port):
    vrep.simxFinish(-1)  # just in case, close all opened connections
    clientID = vrep.simxStart("127.0.0.1", connection_port, True, True, 5000, 5)
    if clientID != -1:
        print("Connected to remote API server")
    else:
        print("Not connected to remote API server")
        sys.exit("Could not connect")

    return clientID


# set properly the linear and angular velocities of the robot
def set_velocity(clientID, forward_back_vel, left_right_vel, rotation_vel, wheel_joints):
    vrep.simxSetJointTargetVelocity(clientID, wheel_joints[0], -forward_back_vel + left_right_vel - rotation_vel,
                                    vrep.simx_opmode_oneshot_wait)
    vrep.simxSetJointTargetVelocity(clientID, wheel_joints[1], -forward_back_vel - left_right_vel + rotation_vel,
                                    vrep.simx_opmode_oneshot_wait)
    vrep.simxSetJointTargetVelocity(clientID, wheel_joints[2], -forward_back_vel + left_right_vel + rotation_vel,
                                    vrep.simx_opmode_oneshot_wait)
    vrep.simxSetJointTargetVelocity(clientID, wheel_joints[3], -forward_back_vel - left_right_vel - rotation_vel,
                                    vrep.simx_opmode_oneshot_wait)


# get the distance between sensor and the nearest object detected (if there ism't an object detected return max_dist)
def get_distance(clientID, sensor, max_dist):
    _, detection_state, detected_point, detected_object_handle, detected_surface_normal_vector = vrep.simxReadProximitySensor(clientID, sensor, vrep.simx_opmode_streaming)
    distance = math.sqrt(math.pow(detected_point[0], 2) + math.pow(detected_point[1], 2) + math.pow(detected_point[2], 2))
    if detection_state < 1 :
        distance = max_dist
    return distance


# get the pose of the object passed as parameter
def get_pose(clientID, object_handle):
    _, position = vrep.simxGetObjectPosition(clientID, object_handle, -1, vrep.simx_opmode_oneshot_wait)
    _, orientation = vrep.simxGetObjectOrientation(clientID, object_handle, -1, vrep.simx_opmode_oneshot_wait)
    return [position[0], position[1], orientation[2]]
