import sim as vrep # access all the VREP elements
import sys
import time
import math


def set_velocity(forward_back_vel, left_right_vel, rotation_vel, wheel_joints):
    vrep.simxSetJointTargetVelocity(clientID, wheel_joints[0], -forward_back_vel + left_right_vel - rotation_vel,
                                    vrep.simx_opmode_oneshot_wait)
    vrep.simxSetJointTargetVelocity(clientID, wheel_joints[1], -forward_back_vel - left_right_vel + rotation_vel,
                                    vrep.simx_opmode_oneshot_wait)
    vrep.simxSetJointTargetVelocity(clientID, wheel_joints[2], -forward_back_vel + left_right_vel + rotation_vel,
                                    vrep.simx_opmode_oneshot_wait)
    vrep.simxSetJointTargetVelocity(clientID, wheel_joints[3], -forward_back_vel - left_right_vel - rotation_vel,
                                    vrep.simx_opmode_oneshot_wait)


def get_distance(sensor, max_dist):
    _, detection_state, detected_point, detected_object_handle, detected_surface_normal_vector = vrep.simxReadProximitySensor(clientID, sensor, vrep.simx_opmode_streaming)
    #_, distance = vrep.simxCheckDistance(clientID, sensor, detected_object_handle, vrep.simx_opmode_streaming)
    distance = math.sqrt(math.pow(detected_point[0], 2) + math.pow(detected_point[1], 2) + math.pow(detected_point[2], 2))
    print(detected_object_handle, detected_point, distance)
    if detection_state < 1 :
        distance = max_dist
    return distance


def get_robot_pose(robot):
    position = vrep.simxGetObjectPosition(clientID, robot, -1)
    orientation = vrep.simxGetObjectOrientation(clientID, robot, -1)
    return [position[1], position[2], orientation[3]]


vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart("127.0.0.1",19999,True,True,5000,5) # start a connection
if clientID!=-1:
    print ("Connected to remote API server")
else:
    print("Not connected to remote API server")
    sys.exit("Could not connect")

error_code, robotHandle = vrep.simxGetObjectHandle(clientID,'youBot',vrep.simx_opmode_oneshot_wait)
print(error_code)
print(robotHandle)
wheel_joints_handle = [-1, -1, -1, -1]
_, wheel_joints_handle[0] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fl', vrep.simx_opmode_oneshot_wait)
_, wheel_joints_handle[1] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fr', vrep.simx_opmode_oneshot_wait)
_, wheel_joints_handle[2] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rr', vrep.simx_opmode_oneshot_wait)
_, wheel_joints_handle[3] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rl', vrep.simx_opmode_oneshot_wait)

[forward_back_vel, left_right_vel, rotation_vel] = [2, 0, 0]
set_velocity(forward_back_vel, left_right_vel, rotation_vel, wheel_joints_handle)

_, sensorHandle = vrep.simxGetObjectHandle(clientID, 'Proximity_sensor', vrep.simx_opmode_oneshot_wait)
print('Sensor Handle: ', sensorHandle)
while 1:
    distance = get_distance(sensorHandle, 1)
    time.sleep(0.5)