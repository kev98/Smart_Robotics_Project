import sim as vrep
import sys
import time
import math
import robotUtils
from tensorflow import keras
import numpy as np
import matplotlib.pyplot as plt
from mobilenet_v1.mobilenet import MobileNet
from finetunedModel import FinetunedModel


def shape_detection(clientID, camera_handle, image_size):
    # get an image from the vision sensor
    _, resolution, curr_img = vrep.simxGetVisionSensorImage(clientID, camera_handle, 0, vrep.simx_opmode_oneshot_wait)
    
    # transform the image in an RGB array (256, 256, 3)
    curr_img = np.array(curr_img, dtype=np.uint8)
    curr_img.resize(resolution[0], resolution[1], 3)
    print(curr_img.shape)

    # rotate and display the image
    curr_img = np.rot90(curr_img, 2)

    # load the already trained finetuned model
    finetuned_model = FinetunedModel(image_size=image_size, model_to_load='models/shape_model.h5')

    plt.imsave('image.jpg', curr_img)
    
    # the actual image currently framed by the vision sensor
    finetuned_model.predict_shape('image.jpg')