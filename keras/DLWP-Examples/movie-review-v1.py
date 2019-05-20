#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May  2 14:20:25 2019

@author: mha114
"""

#%%
from keras.datasets import imdb
(train_data, train_labels), (test_data, test_labels) = imdb.load_data(num_words=10000)

#%%
