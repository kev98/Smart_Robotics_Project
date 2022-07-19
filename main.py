import sim as vrep
import sys
from time import sleep
import math
import robotControl
import robotUtils
import robotVision


if __name__ == '__main__':

    # start a new connection with the CoppeliaSim environment
    clientID = robotUtils.start_connection(19999)
    vrep.simxSetInt32Signal(clientID, "state9", 0, vrep.simx_opmode_oneshot_wait)

    # get the robot and the wheel handles
    _, robotHandle = vrep.simxGetObjectHandle(clientID, 'youBot', vrep.simx_opmode_oneshot_wait)
    print(_, robotHandle)
    wheel_joints_handle = [-1, -1, -1, -1]
    _, wheel_joints_handle[0] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fl', vrep.simx_opmode_oneshot_wait)
    _, wheel_joints_handle[1] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_fr', vrep.simx_opmode_oneshot_wait)
    _, wheel_joints_handle[2] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rr', vrep.simx_opmode_oneshot_wait)
    _, wheel_joints_handle[3] = vrep.simxGetObjectHandle(clientID, 'rollingJoint_rl', vrep.simx_opmode_oneshot_wait)
    print(_, wheel_joints_handle)

    robotUtils.set_velocity(clientID, 0, 0, 0, wheel_joints_handle)

    # get the youBot reference frame handle and its pose wrt world frame
    _, robot_ref = vrep.simxGetObjectHandle(clientID, 'youBot_ref', vrep.simx_opmode_oneshot_wait)
    initial_pos = robotUtils.get_pose(clientID, robot_ref)

    # get the proximity sensor handle and the Lidar sensors handles
    _, sensorHandle = vrep.simxGetObjectHandle(clientID, 'Proximity_sensor', vrep.simx_opmode_oneshot_wait)

    # get the vision sensor handle 
    _, cam_handle = vrep.simxGetObjectHandle(clientID, 'Vision_sensor', vrep.simx_opmode_oneshot_wait)

    flagHandle = []
    flag_pos = []
    num_target = 15

    # get the flag handles and the flag fixed positions wrt world frame
    for i in range(num_target):
        name = 'goal' + str(i)
        print(name)
        _, flagH = vrep.simxGetObjectHandle(clientID, name, vrep.simx_opmode_oneshot_wait)
        flagHandle.append(flagH)
        flagP = robotUtils.get_pose(clientID, flagH)
        flag_pos.append(flagP)

    x_upper = 0.5 - 0.03
    x_lower = 0.5 + 0.03

    for i in range(num_target):
        # initialization of some useful variables
        state = 1
        curr_velocity = [forward_back_vel, left_right_vel, rotation_vel] = [0, 0, 0]
        r = 1
        check5 = False

        while 1:
            vrep.simxSetInt32Signal(clientID, "state8", 0, vrep.simx_opmode_oneshot_wait)
            _, n_blob = vrep.simxGetFloatSignal(clientID, "n_blob", vrep.simx_opmode_oneshot_wait)
            # check if there is an object pointed by the camera
            if n_blob != 0. and state < 8:
                state = 7

            # get the pose of the robot and the detection of something in front of the proximity sensor
            pose = robotUtils.get_pose(clientID, robot_ref)
            dist = robotUtils.get_distance(clientID, sensorHandle, 1)

            # state 1: the robot look for the correct orientation to reach the next target
            if state == 1:
                print(state)
                is_pointing, dir = robotControl.pointing_to_goal(pose, flag_pos[i], 0.02)
                if is_pointing:
                    state = 2
                else:
                    robotControl.compute_turn(clientID, 1, dir, wheel_joints_handle)

            # state 2: the robot goes to the current target
            elif state == 2:
                print(state)
                if robotControl.goal_reached(pose, flag_pos[i], 0.5):
                    state = 6
                elif robotControl.obstacle_in_front(dist, 0.7):
                    state = 3
                else:
                    robotUtils.set_velocity(clientID, 4, 0, 0, wheel_joints_handle)

            # state 3: the robot escape from an obstacle (e.g. a wall)
            elif state == 3:
                print(state)
                robotUtils.set_velocity(clientID, 0, 0, 0, wheel_joints_handle)
                break

            # state 6: the robot has reached the current target, so it can pass to the next one
            elif state == 6:
                print(state)
                robotUtils.set_velocity(clientID, 0, 0, 0, wheel_joints_handle)
                break

            # state 7: reaching the object detected while keeping the blob in the center of the camera
            elif state == 7:
                print(state)
                if dist < 0.7:
                    state = 8
                _, x_target = vrep.simxGetFloatSignal(clientID, "x", vrep.simx_opmode_oneshot_wait)
                _, y_target = vrep.simxGetFloatSignal(clientID, "y", vrep.simx_opmode_oneshot_wait)
                if (x_target) <= x_upper:
                    robotUtils.set_velocity(clientID, 0, 0, -0.5, wheel_joints_handle)
                elif (x_target) >= x_lower:
                    robotUtils.set_velocity(clientID, 0, 0, 0.5, wheel_joints_handle)
                else:
                    robotUtils.set_velocity(clientID, 1, 0, 0, wheel_joints_handle)

            # state 8: shape detection
            elif state == 8:
                print(state)
                vrep.simxSetInt32Signal(clientID, "state8", -1, vrep.simx_opmode_oneshot_wait)
                robotUtils.set_velocity(clientID, 0, 0, 0, wheel_joints_handle)
                predicted_shape = robotVision.shape_detection(clientID, cam_handle, 224)
                print(predicted_shape)
                if predicted_shape == 'Cube':
                    vrep.simxSetStringSignal(clientID, "shape", "cube", vrep.simx_opmode_oneshot_wait)
                elif predicted_shape == 'Spheroid':
                    vrep.simxSetStringSignal(clientID, "shape", "spheroid", vrep.simx_opmode_oneshot_wait)
                state = 9

            # state 9: pick and place
            elif state == 9:
                print(state)
                vrep.simxSetInt32Signal(clientID, "state9", -1, vrep.simx_opmode_oneshot_wait)
                _, done = vrep.simxGetInt32Signal(clientID, "done", vrep.simx_opmode_oneshot_wait)
                if done == 1:
                    vrep.simxSetInt32Signal(clientID, "state9", 0, vrep.simx_opmode_oneshot_wait)
                    state = 10

            # state 10: repositioning in order to return to the current flag
            elif state == 10:
                print(state)
                robotUtils.set_velocity(clientID, 0, 0, 0.5, wheel_joints_handle)
                sleep(15)
                state = 1

        # set the new initial pose
        initial_pos = pose

    # all the tasks are finished, so close the connection
    vrep.simxFinish(-1)
