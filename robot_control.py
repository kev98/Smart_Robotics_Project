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

# NON FUNZIONANO LA SIMXGETOBJECTPOSITION E SIMXGETOBJECTORIENTATION (ritornano sempre [0, 0, 0])
def get_robot_pose(robot):
    _, position = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_buffer)
    _, orientation = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_buffer)
    print(position, orientation)
    return [position[0], position[1], orientation[2]]


# function that verifies if the robot is pointing to the goal
def pointing_to_goal(pose, initial_pos, goal_pos, tolerance):
    angle_diff = math.atan2(goal_pos[2]-initial_pos[2], goal_pos[1]-initial_pos[1])-pose[3]
    if math.fabs(angle_diff) < tolerance:
        pointing = True
    else:
        pointing = False

    if angle_diff > 0:
        # return 1 if a left turn is required
        direction = 1
    else:
        # return -1 if a right turn is required
        direction = -1

    return pointing, direction


def randomness():
    pass


# direction 1 --> follow left, direction -1 --> follow right QUUESTA FUNZIONE PROBABILMENTE NON SERVE A NULLA
def precompute_turn(rotation_vel, direction, wheel_joints, b):
    if direction == 1:
        set_velocity(0, 0, -rotation_vel, wheel_joints) # -rotation_vel because I want a left rotation
    if direction == -1:
        set_velocity(0, 0, rotation_vel, wheel_joints)

    t = b*(math.pi/2)/rotation_vel
    return t


# function to understand if the robot is near an obstacle
def obstacle_detected(direction, dist_fl, dist_rl, dist_fr, dist_rr, dist_obstacle, tolerance):
    if direction == 1:
        dist_f = dist_fr
        dist_r = dist_rr
    else:
        dist_f = dist_fl
        dist_r = dist_rl

    front = math.fabs(dist_f - dist_obstacle) < tolerance  # return true if the obstacle is near the front of the robot
    rear = math.fabs(dist_r - dist_obstacle) < tolerance  # return true if the obstacle is near the rear of the robot

    return front, rear


def follow_obstacle():
    pass


# return -1 if the robot is in the right side of the line that connects the start and the goal point,
# 1 if is in the left side
def line_side():
    pass


# compare position of the robot and the goal to understand if the robot has reached the goal
def goal_reached():
    pass


vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart("127.0.0.1",19999,True,True,5000,5) # start a connection
if clientID != -1:
    print ("Connected to remote API server")
else:
    print("Not connected to remote API server")
    sys.exit("Could not connect")

error_code, robotHandle = vrep.simxGetObjectHandle(clientID,'youBot',vrep.simx_opmode_oneshot_wait)
print(error_code) # se stampa 0 è andato a buon fine
print(robotHandle)
wheel_joints_handle = [-1, -1, -1, -1]
_, wheel_joints_handle[0] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fl', vrep.simx_opmode_oneshot_wait)
_, wheel_joints_handle[1] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fr', vrep.simx_opmode_oneshot_wait)
_, wheel_joints_handle[2] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rr', vrep.simx_opmode_oneshot_wait)
_, wheel_joints_handle[3] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rl', vrep.simx_opmode_oneshot_wait)

curr_velocity = [forward_back_vel, left_right_vel, rotation_vel] = [2, 0, 0]
set_velocity(forward_back_vel, left_right_vel, rotation_vel, wheel_joints_handle)

_, sensorHandle = vrep.simxGetObjectHandle(clientID, 'Proximity_sensor', vrep.simx_opmode_oneshot_wait)
print('Sensor Handle: ', sensorHandle)

'''while 1:
    distance = get_distance(sensorHandle, 1)
    time.sleep(0.5)
'''

time.sleep(2)
set_velocity(0, 0, 2, wheel_joints_handle)
pose = get_robot_pose(robotHandle)

# fino a qui ci sono delle prove, qua inizia il codice dell'obstacle avoidance
# initializations
state = 1





