import sim as vrep
import sys
import time
import math
import robotControl
import robotUtils
import robotVision
import matplotlib.pyplot as plt
import numpy as np

vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
if clientID != -1:
    print("Connected to remote API server")
else:
    print("Not connected to remote API server")
    sys.exit("Could not connect")

_, robot_handle =vrep.simxGetObjectHandle(clientID, '/pioneer', vrep.simx_opmode_oneshot_wait)
_, caster_handle = vrep.simxGetObjectHandle(clientID, 'caster', vrep.simx_opmode_oneshot_wait)
_, right_wheel_handle = vrep.simxGetObjectHandle(clientID, '/pioneer/rightMotor', vrep.simx_opmode_oneshot_wait)
_, left_wheel_handle = vrep.simxGetObjectHandle(clientID, '/pioneer/leftMotor', vrep.simx_opmode_oneshot_wait)

#vrep.simxSetJointTargetVelocity(clientID, caster_handle, 0, vrep.simx_opmode_oneshot_wait)
code = vrep.simxSetJointTargetVelocity(clientID, right_wheel_handle, 0, vrep.simx_opmode_oneshot_wait)
code = vrep.simxSetJointTargetVelocity(clientID, left_wheel_handle, 0, vrep.simx_opmode_oneshot_wait)


_, camera_handle = vrep.simxGetObjectHandle(clientID, '/pioneer/Vision_sensor', vrep.simx_opmode_oneshot_wait)

errprCode,resolution,image = vrep.simxGetVisionSensorImage(clientID,camera_handle,0,vrep.simx_opmode_oneshot_wait)
time.sleep(0.2)
errprCode,resolution1,image1 = vrep.simxGetVisionSensorImage(clientID,camera_handle,0,vrep.simx_opmode_oneshot_wait)

#code, state, pack = vrep.simxReadVisionSensor(clientID, camera_handle, vrep.simx_opmode_oneshot_wait)
#print(pack[5])
#print(pack[6])

#result,pack1,pack2 = vrep.simxReadVisionSensor(clientID, camera_handle, vrep.simx_opmode_oneshot_wait)
#print(result, pack1, pack2)

im = np.array(image, dtype = np.uint8)
im = im.reshape([resolution[0],resolution[1],3])
plt.imshow(im, origin='lower') 
plt.show()

im = np.array(image1, dtype = np.uint8)
im = im.reshape([resolution1[0],resolution1[1],3])
plt.imshow(im, origin='lower') 
plt.show()

emptybuffer = bytearray()
code, outInt, outFloat, OutString, outBuffer = vrep.simxCallScriptFunction(clientID, "pioneer/Vision_sensor", vrep.sim_scripttype_childscript,
            "legacyRemoteApi_movementDataFunction", [], [], [], emptybuffer, operationMode=vrep.simx_opmode_oneshot)

_, x_target = vrep.simxGetFloatSignal(clientID, "x", vrep.simx_opmode_oneshot_wait)
_, y_target = vrep.simxGetFloatSignal(clientID, "y", vrep.simx_opmode_oneshot_wait)

print(code)
print(outInt)
print(outFloat)
print(OutString)
print(outBuffer)

print(x_target)
print(y_target)
