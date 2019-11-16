from __future__ import print_function
import numpy as np
import matplotlib.pyplot as plt

import pandas as pd
import keras
from keras.utils import to_categorical
import os
from keras.preprocessing.image import ImageDataGenerator, load_img

from keras.applications import ResNet50

#importing a pretrained model for accuracy
resnet50_model = ResNet50(weights='imagenet',
                  include_top=False,   ##this means that we don't load the last 2 layers, 
					 	 ##because this are the ones we want 
						 ##to train for our specific purpose
                  input_shape=(224, 224, 3))


for layer in resnet50_model.layers[:-4]:
    layer.trainable = False

for layer in resnet50_model.layers:
    print(layer, layer.trainable)

from keras import models
from keras import layers
from keras import optimizers

model = models.Sequential()
model.add(resnet50_model) ##

model.add(layers.Flatten())
model.add(layers.Dense(2048, activation='relu', input_dim=7 * 7 * 2048))
model.add(layers.Dropout(0.5))
#for regression networks it needs only one unit, and a linear activation
model.add(layers.Dense(1, activation='linear')) 

#just to see the format of the layers
model.summary()


train_dir = 'finall2/training'
validation_dir = 'finall2/validation'
nTrain = 2617 
nVal = 406
batch_size = 32

train_datagen = ImageDataGenerator(rescale=1./255)
validation_datagen = ImageDataGenerator(rescale=1./255)

# Change the batchsize according to your system RAM
train_batch_size = 32
val_batch_size = 16

#search in keras documentation for how the data of flow_from_dataframe works, and the parameters
dataframe_train=pd.read_excel('train_truths.xls')

dataframe_valid=pd.read_excel('validation_truths.xlsx')

train_generator = train_datagen.flow_from_dataframe(
    dataframe_train,
    train_dir,
    x_col="image", #names of the columns from dataframe
    y_col="value",
    target_size=(224, 224),
    batch_size=train_batch_size,
    class_mode='raw') #because we have a regression type network and we associate numbers to images

validation_generator = validation_datagen.flow_from_dataframe(
    dataframe_valid,
    validation_dir,
    x_col="image",
    y_col="value",
    target_size=(224, 224),
    batch_size=val_batch_size,
    class_mode='raw',
    shuffle=False)

model.compile(optimizer=optimizers.Adam(lr=2e-4),
              loss='mean_squared_error', #for gaussian distributions around small numbers, 
					 #but there are other 2 types: 'Mean Squared Logarithmic Error Loss' for big and spread values
					 #and 'Mean Absolute Error Loss' for mostly gaussian distribution but with outliers, 
					 #more info:https://machinelearningmastery.com/how-to-choose-loss-functions-when-training-deep-learning-neural-networks/ 
              metrics=['acc']) #not so important, just as comparison, accuracy

history = model.fit_generator(train_features,
                    train_generator,
		    steps_per_epoch=train_generator.samples/train_generator.batch_size ,
                    epochs=20,
                    validation_data=validation_generator,
		    validation_steps=validation_generator.samples/validation_generator.batch_size,
		    verbose=1)

model.save('third_save.h5')

#ploting a graph of mse evolution
acc = history.history['acc']
val_acc = history.history['val_acc']
loss = history.history['loss']
val_loss = history.history['val_loss']
 
epochs = range(len(acc))
 
plt.plot(epochs, acc, 'b', label='Training acc')
plt.plot(epochs, val_acc, 'r', label='Validation acc')
plt.title('Training and validation accuracy')
plt.legend()
 
plt.figure()
 
plt.plot(epochs, loss, 'b', label='Training loss')
plt.plot(epochs, val_loss, 'r', label='Validation loss')
plt.title('Training and validation loss')
plt.legend()
 
plt.show()
