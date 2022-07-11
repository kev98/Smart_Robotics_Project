from tensorflow import keras
from tensorflow.keras.preprocessing.image import ImageDataGenerator
import matplotlib.pyplot as plt
from finetunedModel import FinetunedModel
from mobilenet_v1.mobilenet import MobileNet
from tensorflow.keras.applications.mobilenet import preprocess_input

# Hyperparameters
FC_LAYERS = [100, 50]
dropout = 0.5
IMAGE_SIZE = 224
ALPHA = 0.75
EPOCHS = 15
model_path = 'models/shape_model5.h5'

# Using MobileNetv1
base_model = MobileNet(input_shape=(IMAGE_SIZE, IMAGE_SIZE,3), alpha = ALPHA,
                     depth_multiplier = 1, dropout = 0.001, include_top = False,
                     weights = "imagenet", classes = 4, backend=keras.backend,
                     layers=keras.layers,models=keras.models,utils=keras.utils)

# Instantiate a finetuned_model
finetuned_model = FinetunedModel(image_size=IMAGE_SIZE, base_model=base_model, dropout=dropout, fc_layers=FC_LAYERS, num_classes=4)

# DataLoader to generate the training set for the fine-tuning from the 3D_Shapes_Dataset
train_datagen = ImageDataGenerator(preprocessing_function=preprocess_input)
train_generator = train_datagen.flow_from_directory('3D_Shapes_Dataset',
                                                 target_size=(IMAGE_SIZE,IMAGE_SIZE),
                                                 color_mode='rgb',
                                                 batch_size=32,
                                                 class_mode='categorical', shuffle=True)

# make the fine-tuning on the 3D_Shapes_Dataset
finetuned_model.model.summary()
finetuned_model.model.compile(optimizer='Adam',loss='categorical_crossentropy',metrics=['accuracy'])
step_size_train=train_generator.n//train_generator.batch_size
history = finetuned_model.model.fit_generator(generator=train_generator,steps_per_epoch=step_size_train,epochs=EPOCHS, shuffle=True)

# save the finetuned model
finetuned_model.model.save(model_path)

# show the results of each training epoch and the final graphs of accuracies and losses evolution
plt.plot(history.history['loss'])
plt.plot(history.history['accuracy'])
plt.title('model loss')
plt.ylabel('loss')
plt.xlabel('epoch')
plt.legend(['loss', 'accuracy'], loc='upper left')
plt.show()
