from tensorflow import keras
import numpy as np
from tensorflow.keras.preprocessing import image
from tensorflow.keras.models import Model
from tensorflow.keras.layers import Dense, GlobalAveragePooling2D, Dropout,Flatten
import matplotlib.pyplot as plt
import cv2


# static function to prepare an image (saved in a file) before feeding the Neural Network with it
def prepare_image(file, IMAGE_SIZE):
    img_path = ''
    img = image.load_img(img_path + file, target_size=(IMAGE_SIZE, IMAGE_SIZE))
    img_array = image.img_to_array(img)
    img_array_expanded_dims = np.expand_dims(img_array, axis=0)
    return keras.applications.mobilenet.preprocess_input(img_array_expanded_dims)


# static function to prepare an image (saved in an array) before feeding the Neural Network with it
def prepare_image_from_array(file, IMAGE_SIZE):
    img_array_expanded_dims = np.expand_dims(file, axis=0)
    return keras.applications.mobilenet.preprocess_input(img_array_expanded_dims)

# class that define a finetuned model of a MobileNet v1
class FinetunedModel():

    def __init__(self, image_size, model_to_load=None, base_model=None, dropout=0, fc_layers=[], num_classes=0):
        if model_to_load is None:

            self.base_model = base_model
            self.num_classes = num_classes

            for layer in base_model.layers:
                layer.trainable = False

            x = base_model.output
            x = GlobalAveragePooling2D()(x)

            for fc in fc_layers:
                # New FC layer, random init
                x = Dense(fc, activation='relu')(x)
                x = Dropout(dropout)(x)

            # New softmax layer
            self.predictions = Dense(num_classes, activation='softmax')(x)

            self.model = Model(inputs=self.base_model.input, outputs=self.predictions)

        else:
            self.model = keras.models.load_model(model_to_load)

        self.image_size = image_size

    # forward function (img: str or np.array)
    def predict_shape(self, img):
        if isinstance(img, str):
            preprocessed_image = prepare_image(img, self.image_size)
            image = cv2.cvtColor(cv2.imread(img), cv2.COLOR_BGR2RGB)
        else:
            preprocessed_image = prepare_image_from_array(img, self.image_size)
            image = img

        predictions_shape = self.model.predict(preprocessed_image)
        labels = ['Cube', 'Cylinder', 'Spheroid', 'Sphere']
        # print("Input Image :")
        plt.imshow(image)
        print("Shape Detected: ", labels[predictions_shape[0].tolist().index(max(predictions_shape[0]))])

