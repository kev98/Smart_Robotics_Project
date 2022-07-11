import sim as vrep
import robotUtils
from tensorflow import keras
import numpy as np
import matplotlib.pyplot as plt
from mobilenet_v1.mobilenet import MobileNet
from finetunedModel import FinetunedModel
from PIL import Image

clientID = robotUtils.start_connection(19999)

# get the vision sensor handle
_, camera_handle = vrep.simxGetObjectHandle(clientID, 'Vision_sensor', vrep.simx_opmode_oneshot_wait)
print(_, camera_handle)

# get an image from the vision sensor
_, resolution, curr_img = vrep.simxGetVisionSensorImage(clientID, camera_handle, 0, vrep.simx_opmode_oneshot_wait)
print(_, resolution, type(curr_img))

# transform the image in an RGB array (256, 256, 3)
curr_img = np.array(curr_img, dtype=np.uint8)
curr_img.resize(resolution[0], resolution[1], 3)

# rotate and display the image
curr_img = np.rot90(curr_img, 2)
plt.imshow(curr_img)
# plt.imshow(curr_img, origin="lower")
plt.show()

# Hyperparameters
IMAGE_SIZE = 224
ALPHA = 0.75
EPOCHS = 20
FC_LAYERS = [100, 50]
dropout = 0.5

# import pre-trained mobileNet
'''import zipfile as zf
files = zf.ZipFile("mobilenet_v1.zip", 'r')
files.extractall('mobilenet_v1')
files.close()'''

# Using MobileNetv1
base_model = MobileNet(input_shape=(IMAGE_SIZE, IMAGE_SIZE,3), alpha = ALPHA,
                     depth_multiplier = 1, dropout = 0.001, include_top = False,
                     weights = "imagenet", classes = 4, backend=keras.backend,
                     layers=keras.layers,models=keras.models,utils=keras.utils)

# load the already trained finetuned model
finetuned_model = FinetunedModel(image_size=IMAGE_SIZE, model_to_load='models/shape_model.h5')

# some test from image previously taken by camera sensor

# plt.imsave('sphere.jpg', imager)
finetuned_model.predict_shape('test_shapes/sphere.jpg')

# plt.imsave('cylinder.jpg', imager)
finetuned_model.predict_shape('test_shapes/cylinder.jpg')

# plt.imsave('spheroid.jpg', imager)
finetuned_model.predict_shape('test_shapes/spheroid.jpg')

# plt.imsave('cube.jpg', imager)
finetuned_model.predict_shape('test_shapes/cube.jpg')

# the actual image currently framed by the vision sensor
finetuned_model.predict_shape(curr_img)




