import os
import csv
import tensorflow as tf
#Loading data
#Get the absolute paths for data
data_path = '/home/workspace/data/driving_log.csv' 
data_path_2 = '/home/workspace/data/driving_log2.csv'
data_path_3 = '/home/workspace/data/driving_log3.csv'
data_path_4 = '/home/workspace/data/driving_log4.csv'
data_path_5 = '/home/workspace/data/driving_log5.csv'
data_path_6 = '/home/workspace/data/driving_log6.csv'
data_path_7 = '/home/workspace/data/driving_log7.csv'
data_path_8 = '/home/workspace/data/driving_log8.csv'
data_path_9 = '/home/workspace/data/driving_log9.csv'
data_path_10 = '/home/workspace/data/driving_log10.csv'
data_path_11 = '/home/workspace/data/driving_log11.csv'

# Load data from csv files
def getdata(path):
    samples = []
    with open(path) as csvfile:
        reader = csv.reader(csvfile)
        for line in reader:
            samples.append(line)
    return samples

# get and merge samples
samples = getdata(data_path) 
samples_2 = getdata(data_path_2)
samples_3 = getdata(data_path_3)
samples_4 = getdata(data_path_4)
samples_5 = getdata(data_path_5)
samples_6 = getdata(data_path_6)
samples_7 = getdata(data_path_7)
samples_8 = getdata(data_path_8)
samples_9 = getdata(data_path_9)
samples_10 = getdata(data_path_10)
samples_11 = getdata(data_path_11)

# Delete the row of the header from samples_2
# As sample_2 is the data prepared by Udacity as default and it has the header. Other data that I obtained don't.
samples_2.pop(0)
samples = samples + samples_2 + samples_3 + samples_4 + samples_5 + samples_6 + samples_7 + samples_8 + samples_9 + samples_10 + samples_11

# Import necessary modules
from sklearn.model_selection import train_test_split
train_samples, validation_samples = train_test_split(samples, test_size=0.2)

import cv2
import numpy as np
import sklearn

# Define generator
def generator(samples, batch_size=32):
    num_samples = len(samples)
    while 1: # Loop forever so the generator never terminates
        sklearn.utils.shuffle(samples)
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]

            images = []
            angles = []
            for batch_sample in batch_samples:
                name = '/home/workspace/data/IMG/'+batch_sample[0].split('/')[-1]
                center_image = cv2.imread(name)
                center_angle = float(batch_sample[3])
                images.append(center_image)
                angles.append(center_angle)

            # trim image to only see section with road
            X_train = np.array(images)
            y_train = np.array(angles)
            yield sklearn.utils.shuffle(X_train, y_train)

# Set our batch size
batch_size=32

# Set input image shape
row = 160
col = 320
ch = 3

print("row is", row)
print("col is", col)
print("ch is", ch)

# compile and train the model using the generator function
train_generator = generator(train_samples, batch_size=batch_size)
validation_generator = generator(validation_samples, batch_size=batch_size)

#Transfer model 
from keras.applications.vgg16 import VGG16 
from keras.backend import tf as ktf
from keras import backend as K
from keras.layers.core import Flatten,Dense
from keras.layers import Lambda, Input, Cropping2D
from keras.models import Model
import math

inp = Input(shape=(row, col,ch))
# Crop data
cut_inp = Cropping2D(cropping=((50,20), (0,0)), input_shape=(row,col,ch))(inp)
# Resize the data
resize = Lambda(lambda image: K.tf.image.resize_images(image, (160, 160)))(cut_inp)
# Preprocess incoming data, centered around zero with small standard deviation 
Stadadization = Lambda(lambda x: x/127.5 - 1.,input_shape=(160, 160, ch),output_shape=(160, 160, ch))(resize)
# Transfer VGG model without last fc layers 
model_VGG = VGG16(weights ='imagenet',include_top=False)(Stadadization)
# Flatten the last layer
flatten = Flatten()(model_VGG)
# Output 1 element
out = Dense(1)(flatten)

model = Model(inputs = inp, outputs = out)

# model check
model.summary()
# Comple and optimization
model.compile(loss='mse', optimizer='adam')
model.fit_generator(train_generator, 
           steps_per_epoch=math.ceil(len(train_samples)/batch_size),
            validation_data=validation_generator, 
            validation_steps=math.ceil(len(validation_samples)/batch_size),
            epochs=1, verbose=1)
# Save the model
model.save('model.h5')