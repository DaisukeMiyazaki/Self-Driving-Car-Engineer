import os
import numpy as np
import pandas as pd
import random
import cv2
import csv
import math
import glob

import tensorflow as tf
from keras import backend as K
from keras import models, optimizers
from keras.preprocessing import image
from keras.preprocessing.image import ImageDataGenerator
from keras.callbacks import ModelCheckpoint

from keras.models import Model
import matplotlib.pyplot as plt

from keras.utils.np_utils import to_categorical
from keras.utils import plot_model
from keras.layers import Conv2D, Dense, Dropout, Flatten, Lambda, Activation, MaxPooling2D, Reshape, Input, concatenate

# Fix the random number
from numpy.random import seed
seed(1)
from tensorflow import set_random_seed
set_random_seed(2)

BATCH_SIZE = 32
EPOCHS = 60
NUM_CLASSES = 4

IMG_HEIGHT = 600
IMG_WIDTH = 800
IMG_CH = 3

row = IMG_HEIGHT
col = IMG_WIDTH
ch = IMG_CH

# changing parameters for log
FCS_LYR_NUM = 14
c_FILE = 51
Target_path = '/home/student/CarND-Capstone/train_nn/nn_visual/wmodel_FeatureMap_{}'
IMG_PATH_TUNE = '/home/student/CarND-Capstone/train_nn/data/UNKNOWN_train/image_59.png'

def resize_image(image):
    return cv2.resize(image,(IMG_WIDTH,IMG_HEIGHT))

def get_image(single_row_data):
    
    image = cv2.imread(single_row_data[0])
    image = resize_image(image)
    label = int(single_row_data[1])

    return image,label

def generator(data):
    num_samples = len(data)

    while True:
        for offset in range(0,num_samples, BATCH_SIZE):
            batch_samples = data[offset:offset + BATCH_SIZE]
            
            images = []
            labels = []
            
            for batch_sample in batch_samples:
            
                image,label = get_image(batch_sample)
                images.append(image)
                labels.append(label)
                	
            x_train = np.array(images)
            y_train = np.array(labels)

            y_train = to_categorical(y_train,num_classes=NUM_CLASSES)
            yield (x_train,y_train)

def get_model(LOG_PATH):
    
    
    inp = Input(shape = (row,col,ch))
    
    resize = Lambda(lambda image: K.tf.image.resize_images(image,(96,96)))(inp)

    Standadization = Lambda(lambda x: x/127.5 - 1.,input_shape=(96,96,ch),output_shape=(96,96,ch))(resize)
 
    conv0 = Conv2D(8,kernel_size=(3,3),activation="relu")(Standadization)
    pool0 = MaxPooling2D(pool_size=(2,2))(conv0)
    dp0 = Dropout(0.25)(pool0)
    
    conv1 = Conv2D(16,kernel_size=(3,3),activation="relu")(dp0)
    pool1 = MaxPooling2D(pool_size=(2,2))(conv1)
    dp1 = Dropout(0.25)(pool1)
    
    conv1 = Conv2D(32,kernel_size=(3,3),activation="relu")(dp1)
    pool2 = MaxPooling2D(pool_size=(2,2))(conv1)
    dp2 = Dropout(0.25)(pool2)
    
    conv3 = Conv2D(64,kernel_size=(3,3),activation="relu")(dp2)
    pool3 = MaxPooling2D(pool_size=(2,2))(conv3)
    dp3 = Dropout(0.25)(pool3)
    
    #conv3 = Conv2D(48,kernel_size=(3,3),activation="relu")(dp2)
    
    dns1 = Dense(128,activation="relu")(dp3)
    flatten = Flatten()(dns1)
    
    
    #dns2 = Dense(160,activation="relu")(dns1)
    
    
    OUTPUT = Dense(4)(flatten)  
    
    """
    inp = Input(shape = (row,col,ch))
    
    resize = Lambda(lambda image: K.tf.image.resize_images(image,(100,75)))(inp)

    Standadization = Lambda(lambda x: x/127.5 - 1.,input_shape=(100,75,ch),output_shape=(100,75,ch))(resize)
 
    #conv0 = Conv2D(4,kernel_size=(3,3),activation="relu")(Standadization)
    #pool0 = MaxPooling2D(pool_size=(2,2))(conv0)
    
    conv1 = Conv2D(8,kernel_size=(3,3),activation="relu")(Standadization)
    pool1 = MaxPooling2D(pool_size=(2,2))(conv1)
    
    dp0 = Dropout(0.25)(pool1)
    
    conv2 = Conv2D(16,kernel_size=(3,3),activation="relu")(conv1)
    pool2 = MaxPooling2D(pool_size=(2,2))(conv2)
    
    #dp1 = Dropout(0.25)(pool2)
    conv3 = Conv2D(32,kernel_size=(3,3),activation="relu")(pool2)
    #pool3 = MaxPooling2D(pool_size=(2,2))(conv3)
    
    
    #conv4 = Conv2D(64,kernel_size=(3,3),activation="relu")(pool3)
    #pool3 = MaxPooling2D(pool_size=(2,2))(conv3)
    
    #dp1 = Dropout(0.25)(pool3)
    
    #dns = Dense(128,activation="relu")(dp1)
    #dp3 = Dropout(0.25)(dns)
    
    flatten = Flatten()(conv3)
    
    OUTPUT = Dense(4)(flatten)  
    """
    model = Model(inputs = inp, outputs = OUTPUT)
    
    
    # write a log
    with open(LOG_PATH,"w") as fh:
        model.summary(print_fn = lambda x: fh.write(x + '\n'))
    
    model.compile(optimizer='adam', loss = 'mse',metrics=['acc'])
    return model, conv1
    
def load_image(path):
    # turn an image at path to a 4-dim tensor 
    img = image.load_img(path,target_size=(600,800))
    img_tensor = image.img_to_array(img)
    img_tensor = np.expand_dims(img_tensor, axis = 0)
    img_tensor /= 255
    
    plt.ion()
    plt.imshow(img_tensor[0])
    plt.show
    print(img_tensor.shape)
    return img_tensor
    
def plot_models(history,c_FILE):
    # turn on interactive mode
    plt.ion()
    
    #plot_model(model,to_file=PATH)
    
    # protting accuracy for record
    plt.plot(history.history['acc'])
    plt.plot(history.history['val_acc'])
    plt.title("Model Accuracy")
    plt.ylabel('Accuracy')
    plt.xlabel('Epoch')
    plt.legend(['Train','Test'], loc = 'upper left')
    plt.show()
    plt.savefig('/home/student/CarND-Capstone/train_nn/nn_visual/FeatureMap_{}/history_acc'.format(c_FILE))
    
    # protting loss value for record
    plt.plot(history.history['loss'])
    plt.plot(history.history['val_loss'])
    plt.title('Model loss')
    plt.ylabel('Loss')
    plt.xlabel('Epoch')
    plt.legend(['Train','Test'],loc='upper left')
    plt.show()
    plt.savefig('/home/student/CarND-Capstone/train_nn/nn_visual/FeatureMap_{}/history_loss'.format(c_FILE))

def visualize_every_channel(model,ch_num,activations,PATH):
    layer_names = []
    # depending on the number of the channel, range varies
    for layer in model.layers[:ch_num]:
        layer_names.append(layer.name) # names of layers
        
    images_per_row = 4
    c_IMG = 0
    for layer_name, layer_activation in zip(layer_names, activations):
        n_features = layer_activation.shape[-1]
        size = layer_activation.shape[1]
        size_2 = layer_activation.shape[2]
        n_cols = n_features // images_per_row # tiles the activation channels in this matrix
        
        display_grid = np.zeros((size * n_cols, images_per_row * size))
        
        for col in range(n_cols):
            for row in range(images_per_row):
                channel_image = layer_activation[0,
                                                 :,:,
                                                 col * images_per_row + row]
                channel_image -= channel_image.mean()
                channel_image /= channel_image.std()
                channel_image *= 64
                channel_image += 128
                # limit an array's value from 0 to 255
                channel_image = np.clip(channel_image, 0, 255).astype('uint8')
                display_grid[col * size : (col + 1) * size,
                             row * size_2 : (row + 1) * size_2] = channel_image
        scale = 1. / size
        scale_2 = 1. / size_2
        plt.figure(figsize = (scale_2 * display_grid.shape[1],
                              scale * display_grid.shape[0]))
        plt.title(layer_name)
        plt.grid(False)
        plt.imshow(display_grid, aspect='auto', cmap='gray',interpolation='none')
        #plt.imshow(display_grid, aspect='auto', cmap='viridis')
        plt.savefig(PATH.format(c_IMG))
        #plt.savefig("/home/student/CarND-Capstone/train_nn/nn_visual/FeatureMap/Layer_{}.jpg".format(count))
        c_IMG += 1

if __name__ == "__main__":
        
    abs_path = '/home/student/CarND-Capstone/train_nn/data/traffic_light_train.csv' 
    
    if os.path.exists('/home/student/CarND-Capstone/train_nn/data/traffic_light_train.csv'):
        print( "----model data file is ready----")
    else:
        print( "data file empty")
    
    data_list = []
    
    with open(abs_path) as csvfile:
        reader = csv.reader(csvfile)
        for line in reader:
            data_list.append(line)
    data_list.pop(0)
    
    data_list = random.sample(data_list,len(data_list))
    
    d_train = data_list[:int(len(data_list)*0.8)]
    d_valid = data_list[int(len(data_list)*0.8):]
    
    train_gen = generator(d_train)
    validation_gen = generator(d_valid)
    	
    LOG_PATH = Target_path.format(c_FILE) + '/log.txt'
    model, conv1 = get_model(LOG_PATH)
    
    #print( "type of conv variable is :", type(conv1))
    
    print( 'Training started')
    
    """
    # When training, use checkpointer and history
    checkpointer = ModelCheckpoint(filepath="best_weights.hdf5",monitor = "val_acc",
                                   verbose=1,save_best_only=True)
    
    history = model.fit_generator(
        train_gen,
        steps_per_epoch=math.ceil(len(d_train)/BATCH_SIZE),
        epochs=EPOCHS,
        callbacks=[checkpointer],
        validation_data=validation_gen,
        validation_steps=math.ceil(len(d_valid)/BATCH_SIZE),
        verbose=1
    )
    
    """
    model.load_weights('best_weights.hdf5')
    print("weights saved successfully")
    
    # when not training, comment this out
    #model.save('shapes_cnn.h5')
    #print('Model saved successfully') 
    
    
    
    # plot model
    #MODEL_PLT_PATH = Target_path.format(c_FILE) + '/model.png'
    # plot_models(model,history,c_FILE,PATH):
    #plot_models(history,c_FILE)
    
    img_tensor = load_image(IMG_PATH_TUNE)
    
    layer_outputs = [layer.output for layer in model.layers[:FCS_LYR_NUM]]
    # Extracts the outputs of the top 9 layers
    
    activation_model = models.Model(inputs = model.input,outputs = layer_outputs)
    # create a model that will return these outputs, given the model input
    
    activations = activation_model.predict(img_tensor)
    # Returns a list of five Numpy arrays: one array per layer activation
    
    FeatureMap_path = Target_path.format(c_FILE) + '/Layer_{}.jpg'
    visualize_every_channel(model,FCS_LYR_NUM,activations,FeatureMap_path)
    
    # Destroying the current tf graph 
    K.clear_session()
    
