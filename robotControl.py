import math
import robotUtils


# function that verifies if the robot is pointing to the goal
def pointing_to_goal(pose, goal_pos, tolerance):
    beta = pose[2]
    if beta <= -math.pi/2:
        beta = beta + 2*math.pi
    beta = beta + math.pi/2
    alpha = math.atan2(goal_pos[1] - pose[1], goal_pos[0] - pose[0])
    if alpha < 0:
        alpha = alpha + 2*math.pi
    angle_diff = alpha - beta

    if math.fabs(angle_diff) < tolerance:
        pointing = True
    else:
        pointing = False

    if (angle_diff > 0 and angle_diff < math.pi) or (angle_diff < -math.pi):
        direction = 1
    else: 
        direction = -1

    return pointing, direction


# direction 1 --> follow left, direction -1 --> follow right
def compute_turn(clientID, rotation_vel, direction, wheel_joints):
    if direction == 1:
        robotUtils.set_velocity(clientID, 0, 0, -rotation_vel, wheel_joints) # -rotation_vel because I want a left rotation
    if direction == -1:
        robotUtils.set_velocity(clientID, 0, 0, rotation_vel, wheel_joints)


# return true if there is an obstacle closer than the tolerance
def obstacle_in_front(distance, tolerance):
    if distance < tolerance:
        return True
    else:
        return False


# compare position of the robot and the goal to understand if the robot has reached the goal
def goal_reached(pose, goal_pose, tolerance):
    dist = math.sqrt((goal_pose[0]-pose[0])**2+(goal_pose[1]-pose[1])**2)
    return dist < tolerance
