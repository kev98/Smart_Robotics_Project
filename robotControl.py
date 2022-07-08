import math
import robotUtils


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


# direction 1 --> follow left, direction -1 --> follow right
def compute_turn(clientID, rotation_vel, direction, wheel_joints):
    if direction == 1:
        robotUtils.set_velocity(clientID, 0, 0, -rotation_vel, wheel_joints) # -rotation_vel because I want a left rotation
    if direction == -1:
        robotUtils.set_velocity(clientID, 0, 0, rotation_vel, wheel_joints)

    # t = b*(math.pi/2)/rotation_vel
    # return t


# function to understand if the robot is near an obstacle !!NON USATA!!
def is_parallel(lidarHandle, tolerance):
    dF = robotUtils.get_distance(lidarHandle[1], 1)
    dR = robotUtils.get_distance(lidarHandle[2], 1)
    if math.fabs(dF - dR) < tolerance and dF != 1 and dR != 1:
        return True
    else:
        return False


# return true if there is an obstacle closer than the tolerance
def obstacle_in_front(distance, tolerance):
    if distance < tolerance:
        return True
    else:
        return False
    

# return true if a corner of an obstacle is detected !!NON USATA!!
def corner_detected(lidarHandle):
    if robotUtils.get_distance(lidarHandle[1], 1) < 1:
        return False
    else:
        return True


# compare position of the robot and the goal to understand if the robot has reached the goal
def goal_reached(pose, goal_pose, tolerance):
    dist = math.sqrt((goal_pose[0]-pose[0])**2+(goal_pose[1]-pose[1])**2)
    return dist < tolerance
