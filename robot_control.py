from random import random
from zlib import DEFLATED
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
    if detection_state < 1 :
        distance = max_dist
    return distance


def get_robot_pose(robot):
    _, position = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_oneshot_wait)
    _, orientation = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_oneshot_wait)
    return [position[0], position[1], orientation[2]]


# function that verifies if the robot is pointing to the goal
def pointing_to_goal(pose, initial_pos, goal_pos, tolerance):
    if goal_pos[1]-initial_pos[1] < 0 and goal_pos[0] - initial_pos[0] < 0:
        angle_diff = math.atan2(goal_pos[1] - initial_pos[1], goal_pos[0] - initial_pos[0]) - pose[2] + 3*math.pi / 2
    else:
        angle_diff = math.atan2(goal_pos[1] - initial_pos[1], goal_pos[0] - initial_pos[0]) - pose[2] - math.pi / 2
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
    n = random()
    if n < 0.5:
        direction = -1 # right rotation
    else:
        direction = 1 # left rotation
    return direction


# direction 1 --> follow left, direction -1 --> follow right QUUESTA FUNZIONE PROBABILMENTE NON SERVE A NULLA
def precompute_turn(rotation_vel, direction, wheel_joints):
    if direction == 1:
        set_velocity(0, 0, -rotation_vel, wheel_joints) # -rotation_vel because I want a left rotation
    if direction == -1:
        set_velocity(0, 0, rotation_vel, wheel_joints)

    # t = b*(math.pi/2)/rotation_vel
    # return t


# function to understand if the robot is near an obstacle
def obstacle_detected(direction, dist_obstacle, tolerance, lidarHandle):
    if direction == 1:
        dF = get_distance(lidarHandle[1], 1)
        dR = get_distance(lidarHandle[2], 1)
    else:
        dF = get_distance(lidarHandle[0], 1)
        dR = get_distance(lidarHandle[3], 1)

    front = math.fabs(dF - dist_obstacle) < tolerance  
    rear = math.fabs(dR - dist_obstacle) < tolerance
    return front and rear

def obstacle_in_front(distance, tolerance):
    if distance < tolerance:
        return True
    else:
        return False

def velocity(dmin, d, dmax, vmax, alpha, b, e):
    if d >= dmin:
        v = vmax * d / dmax
    else:
        v = 0
    vL = v * (math.cos(alpha) + b / e * math.sin(alpha))
    vR = v * (math.cos(alpha) - b / e * math.sin(alpha))
    return v, vL, vR

def radius_wheels(lidarHandle):
    _, zmin = vrep.simxGetObjectFloatParam(clientID, lidarHandle[0], 17, vrep.simx_opmode_oneshot_wait)
    _, zmax = vrep.simxGetObjectFloatParam(clientID, lidarHandle[0], 20, vrep.simx_opmode_oneshot_wait)
    radius = (zmax - zmin)/2

    return radius

def follow_obstacle(lidarHandle, dWallSide, direction, kWall, e):
    dFR = get_distance(lidarHandle[1], 1)
    dRR = get_distance(lidarHandle[2], 1)
    dFL = get_distance(lidarHandle[0], 1)
    dRL = get_distance(lidarHandle[3], 1)

    _, aL = vrep.simxCheckDistance(clientID, lidarHandle[0], lidarHandle[3], vrep.simx_opmode_oneshot_wait)
    _, aR = vrep.simxCheckDistance(clientID, lidarHandle[1], lidarHandle[2], vrep.simx_opmode_oneshot_wait)

    if direction == 1: # left
        phi = math.atan((dFR-dRR)/aR)
        d = 0.5 * (dFR + dRR) - dWallSide
    else:
        phi = math.atan((dRL-dFL)/aL)
        d = dWallSide - 0.5 * (dFL + dRL)

    gamma = d * kWall
    alpha = phi + gamma
    radius = radius_wheels(lidarHandle)
    _, b = vrep.simxCheckDistance(clientID, lidarHandle[0], lidarHandle[3], vrep.simx_opmode_oneshot_wait)
    b = b / 2
    v, vL, vR = velocity(0, dist, 1, 2, alpha, b, e)
    wL = (v / radius) * (math.cos(alpha) + b / e * math.sin(alpha))
    wR = (v / radius) * (math.cos(alpha) + b / e * math.sin(alpha))
    return wL, wR, vL, vR



# return -1 if the robot is in the right side of the line that connects the start and the goal point,
# 1 if is in the left side
def line_side():
    pass


# compare position of the robot and the goal to understand if the robot has reached the goal
def goal_reached(pose, goal_pose, tolerance):
    dist = math.sqrt((goal_pose[0]-pose[0])**2+(goal_pose[1]-pose[1])**2)
    return dist<tolerance
    pass


vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart("127.0.0.1",19999,True,True,5000,5) # start a connection
if clientID != -1:
    print ("Connected to remote API server")
else:
    print("Not connected to remote API server")
    sys.exit("Could not connect")

error_code, robotHandle = vrep.simxGetObjectHandle(clientID,'youBot',vrep.simx_opmode_oneshot_wait)
wheel_joints_handle = [-1, -1, -1, -1]
_, wheel_joints_handle[0] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fl', vrep.simx_opmode_oneshot_wait)
_, wheel_joints_handle[1] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fr', vrep.simx_opmode_oneshot_wait)
_, wheel_joints_handle[2] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rr', vrep.simx_opmode_oneshot_wait)
_, wheel_joints_handle[3] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rl', vrep.simx_opmode_oneshot_wait)

_, robot_ref = vrep.simxGetObjectHandle(clientID, 'youBot_ref', vrep.simx_opmode_oneshot_wait)
initial_pos = get_robot_pose(robot_ref)
# initial_pos = get_robot_pose(robotHandle)


_, sensorHandle = vrep.simxGetObjectHandle(clientID, 'Proximity_sensor', vrep.simx_opmode_oneshot_wait)
lidarHandle = [-1, -1, -1, -1]
_, lidarHandle[0] = vrep.simxGetObjectHandle(clientID, 'ir_front_left', vrep.simx_opmode_oneshot_wait)
_, lidarHandle[1] = vrep.simxGetObjectHandle(clientID, 'ir_front_right', vrep.simx_opmode_oneshot_wait)
_, lidarHandle[2] = vrep.simxGetObjectHandle(clientID, 'ir_rear_right', vrep.simx_opmode_oneshot_wait)
_, lidarHandle[3] = vrep.simxGetObjectHandle(clientID, 'ir_rear_left', vrep.simx_opmode_oneshot_wait)

'''while 1:
    distance = get_distance(sensorHandle, 1)
    time.sleep(0.5)
'''

time.sleep(2)
# set_velocity(0, 0, 2, wheel_joints_handle)

_, flagHandle = vrep.simxGetObjectHandle(clientID, 'goal', vrep.simx_opmode_oneshot_wait)
flag_pos = get_robot_pose(flagHandle)

'''
while 1:
    pose = get_robot_pose(robot_ref)
    p, direc = pointing_to_goal(pose, initial_pos, flag_pos, 0.5)
    print(pose)
    print(p, direc)
    time.sleep(1)
'''
# fino a qui ci sono delle prove, qua inizia il codice dell'obstacle avoidance
# initializations
state = 1
curr_velocity = [forward_back_vel, left_right_vel, rotation_vel] = [0, 0, 0]

while 1:
    dist = get_distance(sensorHandle, 1)
    pose = get_robot_pose(robot_ref)
    r = randomness()

    if state == 1:
        is_pointing, dir = pointing_to_goal(pose, initial_pos, flag_pos, 0.05)
        if is_pointing:
            state = 2
        else:
            precompute_turn(0.5, dir, wheel_joints_handle, )

    elif state == 2:
        if goal_reached(pose, flag_pos, 0.1):
            state = 6
        elif obstacle_in_front(dist, 0.5):
            state = 3
        elif not is_pointing:
            state = 1
        else:
            set_velocity(2, 0, 0, wheel_joints_handle)

    elif state == 3:
        precompute_turn(0.5, r, wheel_joints_handle)
        if obstacle_detected(r, 0.7, 0.5, lidarHandle):
            state = 4

    elif state == 4:
        wL, wR, vL, vR = follow_obstacle(lidarHandle, 0.5, r, 0.1, 1)
        vrep.simxSetJointTargetVelocity(clientID, wheel_joints_handle[0], vL+wL, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetVelocity(clientID, wheel_joints_handle[1], vR+wR, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetVelocity(clientID, wheel_joints_handle[2], vR+wR, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetVelocity(clientID, wheel_joints_handle[3], vL+wL, vrep.simx_opmode_oneshot_wait)

    elif state == 6:
        set_velocity(0, 0, 0, wheel_joints_handle)