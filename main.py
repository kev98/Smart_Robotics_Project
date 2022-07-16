import sim as vrep
import sys
import time
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
    lidarHandle = [-1, -1, -1, -1]
    _, lidarHandle[0] = vrep.simxGetObjectHandle(clientID, 'ir_front_left', vrep.simx_opmode_oneshot_wait)
    _, lidarHandle[1] = vrep.simxGetObjectHandle(clientID, 'ir_front_right', vrep.simx_opmode_oneshot_wait)
    _, lidarHandle[2] = vrep.simxGetObjectHandle(clientID, 'ir_rear_right', vrep.simx_opmode_oneshot_wait)
    _, lidarHandle[3] = vrep.simxGetObjectHandle(clientID, 'ir_rear_left', vrep.simx_opmode_oneshot_wait)

    # get the vision sensor handle 
    _, cam_handle = vrep.simxGetObjectHandle(clientID, 'Vision_sensor', vrep.simx_opmode_oneshot_wait)

    flagHandle = []
    flag_pos = []
    num_target = 12

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
            if n_blob != 0. and state < 8:
                state = 7

            # get the pose of the robot and the detection of something in front of the proximity sensor
            pose = robotUtils.get_pose(clientID, robot_ref)
            dist = robotUtils.get_distance(clientID, sensorHandle, 1)

            # state 1: the robot look for the correct orientation to reach the next target
            if state == 1:
                print(state)
                is_pointing, dir = robotControl.pointing_to_goal(pose, initial_pos, flag_pos[i], 0.02)
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

            # state 7: 
            elif state == 7:
                print(state, dist)
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

            elif state == 8:
                print(state)
                vrep.simxSetInt32Signal(clientID, "state8", -1, vrep.simx_opmode_oneshot_wait)
                robotUtils.set_velocity(clientID, 0, 0, 0, wheel_joints_handle)
                predicted_shape = robotVision.shape_detection(clientID, cam_handle, 224)
                if predicted_shape == 'cube':
                    vrep.simxSetStringSignal(clientID, "shape", "cube", vrep.simx_opmode_oneshot_wait)
                elif predicted_shape == 'spheroid':
                    vrep.simxSetStringSignal(clientID, "shape", "spheroid", vrep.simx_opmode_oneshot_wait)
                state = 9

            elif state == 9:
                print(state)
                vrep.simxSetInt32Signal(clientID, "state9", -1, vrep.simx_opmode_oneshot_wait)

            '''elif state == 3:
                print(state)
                set_velocity(0, 0, -0.5, wheel_joints_handle)
                if is_parallel(lidarHandle, 0.05):
                    state = 4

            elif state == 4:
                print(state)
                set_velocity(2, 0, 0, wheel_joints_handle)
                if corner_detected(lidarHandle):
                    initial_time = vrep.c_GetLastCmdTime(clientID)
                    state = 5
                m = math.atan2(flag_pos[i][1]-initial_pos[1], flag_pos[i][0]-initial_pos[0])-math.pi/2
                print(pose[1] - m * pose[0] - flag_pos[i][1] + m * flag_pos[i][0])
                if check5 and math.fabs(pose[1]-m*pose[0]-flag_pos[i][1]+m*flag_pos[i][0]) < 0.05:
                    state = 1
                    check5 = False

            elif state == 5:
                print(state)
                check5 = True
                set_velocity(1.7, 0, 1, wheel_joints_handle)
                curr_orientation = get_robot_pose(robot_ref)[2]
                if (vrep.c_GetLastCmdTime(clientID)-initial_time)/1000 > 2 :
                    state = 4
                """if initial_orientation*curr_orientation < 0 and (math.fabs(initial_orientation)-math.fabs(curr_orientation)) < 0.05:
                    state = 4
                elif initial_orientation*curr_orientation > 0 and math.fabs(initial_orientation-curr_orientation-math.pi/2) < 0.05 :
                    state = 4"""'''
        # set the new initial pose
        initial_pos = pose

    # all the tasks are finished, so close the connection
    vrep.simxFinish(-1)
