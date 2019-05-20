# -*- coding: utf-8 -*-
"""
Keras "hello world" - training on MNIST digit dataset
Written using Spyder

2018-09-27 Martin H. Skjelvareid
"""

#%% Force CPU use only (just for comparing performace)
#import os
#os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"   # see issue #152
#os.environ["CUDA_VISIBLE_DEVICES"] = ""

#%% Import data
from keras.datasets import mnist
(train_images, train_labels), (test_images, test_labels) = mnist.load_data()

#%% Inspect training and test data (use F9 to execute line by line)
train_images.shape
len(train_labels)
train_labels

test_images.shape
len(test_labels)
test_labels

#%% Create network
from keras import models
from keras import layers
network = models.Sequential()  
network.add(layers.Dense(512, activation='relu', input_shape=(28 * 28,)))
network.add(layers.Dense(10, activation='softmax'))

#%% Compile the network
network.compile(optimizer='rmsprop',
    loss='categorical_crossentropy',
    metrics=['accuracy'])

#%% Prepare input data (reshape, convert to float, scale)
train_images = train_images.reshape((60000, 28 * 28))
train_images = train_images.astype('float32') / 255
test_images = test_images.reshape((10000, 28 * 28))
test_images = test_images.astype('float32') / 255

#%% Prepare data labels (convert to categorical)
from keras.utils import to_categorical
train_labels = to_categorical(train_labels)
test_labels = to_categorical(test_labels)

#%% Train the network
network.fit(train_images, train_labels, epochs=5, batch_size=128)

#%% Evaluate performance on test data set
test_loss, test_acc = network.evaluate(test_images, test_labels)
test_loss
test_acc
