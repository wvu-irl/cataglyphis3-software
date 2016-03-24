#!/usr/bin/python
'''
AUTHOR: Rahul Kavi
Feature Vector generator/classifier for FishEye images for CATAGLYPHIS
'''
### This program implements a basic convolutional neural
### network based classifier.
### This has 3 convolutional layers, 2 fully connected layers
### and 1 SoftMax layer
from __future__ import division
import theano
from theano import tensor as T
from theano.sandbox.rng_mrg import MRG_RandomStreams as RandomStreams
import numpy as np
from theano.tensor.nnet.conv import conv2d
from theano.tensor.signal.pool import pool_2d
from lasagne.init import GlorotUniform
import os
from os.path import expanduser
import cv2

curDirName = os.path.dirname(os.path.abspath(__file__))

import lmdb
import time
import sys
import traceback
import cPickle
from time import gmtime, strftime

# theano.config.compute_test_value = 'warn' # Use 'warn' to activate this feature

class DeepFishNet:
    '''
    this class represents a convolutional neural network with 3 convolutional layers
    and 2 fully connected layers connected to 1 layer LSTM cells with a final softmax layer. Predicts 5 classes
    '''
    def __init__(self, imgSize = 150, loadData = True, mode= None, modelToLoad=None, randomData=False, dropout_params = None):
        # get your random number generator
        assert(mode != None)
        
        self.mode = mode
        self.srng = RandomStreams()
        self.imgSize = imgSize
        self.dataMatTrain = None
        self.labelMatTrain = None
        self.dataMatTest = None
        self.labelMatTest = None
        self.dropout_params = dropout_params

        self.totalTrainSamples = None
        self.totalTestSamples = None
        self.randomData = None
            
        # initialize your model
        self.initializeModel()

        if(mode == 'Train'):
            print 'calling train your model.....'
            self.trainThisModel()
        elif(mode == 'Test'):
            assert(modelToLoad != None)
            self.loadThisModel(modelToLoad)
            pass
        pass

    def getDataWithIndex(self, start, end, whichData):
        if(whichData == 'TRAIN'):
            try:
                return self.dataMatTrain[start: end], self.labelMatTrain[start: end] # return data[start: end], label[start: end]
            except:
                print '-'*60
                print start, end, 'attempted to get data between ', start, 'and ', end, 'in train'
                traceback.print_exc(file=sys.stdout)
                print '-'*60
        elif(whichData == 'TEST'):
            try:
                return self.dataMatTest[start: end], self.labelMatTest[start: end] # return data[start: end], label[start: end]
            except:
                print '-'*60
                print start, end, 'attempted to get data between ', start, 'and ', end, 'in test'
                traceback.print_exc(file=sys.stdout)
                print '-'*60
        else:
            print 'data has to be TRAIN OR TEST only'
            exit()
        pass

    def matrifyMyData(self):
        '''
        returns labels in matrix form by "matrifying" (some ridiculous name I came up with)
        labels of 0, 1, 1, ... will be replaced by the matrices with their respective index set to 1
        as 
        [1, 0], [0, 1], [0, 1], .....
        '''
        
        nTrainLabel = []
        # nTestLabel = []
        self.uniqueClasses = np.unique(self.labelMatTrain)
        self.nClasses = self.uniqueClasses.shape[0]
        # print "nClasses ",self.nClasses, self.uniqueClasses
        dLabel = np.eye(self.nClasses)

        for i in range(self.totalTrainSamples):
            #print self.labelMatTrain[i][0], dLabel[self.labelMatTrain[i][0]]
            nTrainLabel.append(dLabel[self.labelMatTrain[i][0]])
        nTrainLabel = np.array(nTrainLabel)
        
        self.labelMatTrain = nTrainLabel
        # print 'final data,label shape'
        # print self.labelMatTrain.shape
        # print self.dataMatTrain.shape
        # self.labelMatTest = nTestLabel
        return

    # float casting your input
    def floatX(self, X):
        return np.asarray(X, dtype=theano.config.floatX)

    def init_weights(self, shape, weightType = None, typeLayer = None, caffeLayerName = None):
        return theano.shared(self.floatX(np.random.randn(*shape) * 0.01))

    def sigmoid(self, X):
        return 1.0 / (1.0 + T.exp(-X))

    #   apply non-linear activation function of the given input # aparently ReLU is faster than signoid/tanh
    def rectify(self, X):
        return T.maximum(X, 0.)

    # get your final softmax classifier so that it returns probabilities
    def softmax(self, X):
        # make it mathematically easier
        e_x = T.exp(X - X.max(axis=1).dimshuffle(0, 'x'))
        return e_x / e_x.sum(axis=1).dimshuffle(0, 'x')

    # define dropout function with the given probability of retaining
    def dropout(self, X, p=0.):
        if p > 0:
            retain_prob = 1 - p
            X *= self.srng.binomial(X.shape, p=retain_prob, dtype=theano.config.floatX)
            X /= retain_prob
        return X

    # your objective function cost minimizing function.
    def RMSprop(self, costC, paramsC, lr=0.02, rho=0.9, epsilon=1e-6):
        grads = T.grad(cost=costC, wrt=paramsC)
        updates = []
        ii = 0
        # print len(params)
        # print len(grads)
        for p, g in zip(paramsC, grads):
            # ii += 1
            # print ii, p.get_value()
            acc = theano.shared(p.get_value() * 0.)
            acc_new = rho * acc + (1 - rho) * g ** 2
            gradient_scaling = T.sqrt(acc_new + epsilon)
            g = g / gradient_scaling
            updates.append((acc, acc_new))
            updates.append((p, p - lr * g))
        return updates

    # your main model 

    # 3 convolutional layers
    # flatten the output
    # then pass it to 
    # 3 fully connected layers
    # after 3 layers
    # connect it to softmax layer

    def model(self, X, w1, w2, w3, w4, w5, w_output, p_drop_conv = 0, p_drop_hidden = 0):

        # first convolutional layer
        l1a = self.rectify(conv2d(X, w1, border_mode='full'))
        l1 = pool_2d(l1a, (2, 2), st=None, ignore_border=False, padding=(0, 0), mode='max')
        if(self.mode == "Train"):
            l1 = self.dropout(l1, p_drop_conv)
        # convOut1 = conv2d(X, w1)
        convOut1 = l1a
        
        # second convolutional layer
        l2a = self.rectify(conv2d(l1, w2))
        l2 = pool_2d(l2a, (2, 2), st=None, ignore_border=False, padding=(0, 0), mode='max')
        if(self.mode == "Train"):
            l2 = self.dropout(l2, p_drop_conv)

        # # third convolutional layer
        l3a = self.rectify(conv2d(l2, w3))
        l3 = pool_2d(l3a, (2, 2), st=None, ignore_border=False, padding=(0, 0), mode='max')

        # # flatten the output
        l3 = T.flatten(l3, outdim=2)
        l3 = self.dropout(l3, p_drop_conv)
        if(self.mode == 'Train'):
            l3 = self.dropout(l3, p_drop_conv)

        # 1st fully connected layer
        l4 = self.rectify(T.dot(l3, w4))
        if(self.mode == "Train"):
            l4 = self.dropout(l4, p_drop_hidden)

        # 2nd fully connected layer
        l5 = self.rectify(T.dot(l4, w5))
        if(self.mode == "Train"):
            l5 = self.dropout(l5, p_drop_hidden)

        # connected the above output to softmax layer
        pyx = self.softmax(T.dot(l5, w_output))
        
        return l1, l2, l3, l4, l5, pyx, convOut1

    def initializeModel(self):
        print 'defining model'

        X = T.ftensor4()
        Y = T.fmatrix()

        #initialize your weghts, kernels
        # format n kernels, n channels, kernel_w x kernel_h
        # 20 kernels on gray scale image with 5 x 5 sized kernel
        w1 = self.init_weights((20, 3, 5, 5), weightType = 'Xavier', caffeLayerName = 'conv1')

        # 50 20-channel 5 x 5 sized kernel
        w2 = self.init_weights((50, 20, 5, 5), weightType = 'Xavier', caffeLayerName = 'conv2')

        # 50 50-channel 4 y_x 4 sized kernel
        w3 = self.init_weights((50, 50, 4, 4), weightType = 'Xavier', caffeLayerName = 'conv3')

        # flatten the inputs and pass to fully connected layer
        w4 = self.init_weights((14450, 1000), weightType = 'Xavier')

        # flatten the inputs and pass to fully connected layer
        w5 = self.init_weights((1000, 500), weightType = 'Xavier')
                
        # flatten the inputs and pass to fully connected layer
        w_output = self.init_weights((500, 2), weightType = 'Xavier')

        if(self.dropout_params == None):
            # if there is no default dropout params mentioned, just set them manually
            self.dropout_params = {}
            self.dropout_params['conv'] = 0.1
            self.dropout_params['fc'] = 0.2
        print 'initializing with dropout_params: ', self.dropout_params['conv'], self.dropout_params['fc']
        
        # define your deep model
        noise_l1, noise_l2, noise_l3, noise_l4, noise_l5, noise_py_x, convOut1 = self.model(X, w1, w2, w3, w4, w5, w_output, p_drop_conv = self.dropout_params['conv'], p_drop_hidden = self.dropout_params['fc'])
     
        # get your label from the predicted probabilties
        y_x = T.argmax(noise_py_x, axis=1)

        # for binary cross entropy
        # y_x = noise_py_x >= 0.5

        self.learning_rate = 0.001

        self.cost = T.mean(T.nnet.categorical_crossentropy(noise_py_x, Y))
        # self.cost = T.mean((T.nnet.binary_crossentropy(noise_py_x, Y)))
        
        self.params = [w1, w2, w3, w4, w5, w_output]
        self.paramUpdates = self.RMSprop(self.cost, self.params, lr = self.learning_rate)
        #self.paramUpdates = self.MomentumOptimizer(self.cost, self.params, lr = self.learning_rate)

        print 'compiling functions'
        # print 'current learning rate: ', self.learning_rate
        start_compilation_time = time.clock()
        if(self.mode == "Train"):
            print 'compiling train function startin at ', strftime("%Y-%m-%d %H:%M:%S")
            self.train = theano.function(inputs=[X, Y], outputs=self.cost, updates=self.paramUpdates, allow_input_downcast=True)
        print 'compiling predict function'
        self.predict = theano.function(inputs=[X], outputs=y_x, allow_input_downcast=True)
        print 'compiling predictProb function'
        self.predictProb = theano.function(inputs=[X], outputs=noise_py_x, allow_input_downcast=True)
        end_compilation_time = time.clock()
        self.getFirstLayerOutput = theano.function(inputs=[X], outputs=convOut1)
        # print 'compiled the functions, ended at ', strftime("%Y-%m-%d %H:%M:%S")
        print 'time taken to compile the functions: ', end_compilation_time - start_compilation_time
        
    def saveThisModel(self, fileName = None):
        os.chdir(curDirName)
        params = []
        for eachParam in self.params:
            params.append(eachParam.get_value())
        print len(params)
        print params[0].shape
        # try saving with npz format
        if(fileName == None):
            fileName = 'MyLeNet_'+str(strftime("%Y-%m-%d %H:%M:%S"))+'total_epochs_'+str(self.total_epochs)+'.npz'
        np.savez_compressed(fileName, params = params)
        print 'saving to ', os.getcwd()
        print 'saved to ', fileName
        self.fileName = fileName
        pass
    
    def loadThisModel(self, modelToLoad):
        print 'loading the classifier model'
        print modelToLoad
        # print os.path.exists(modelToLoad) == True
        params = np.load(modelToLoad)
        allParams = params['params']
        # print type(allParams)
        # print allParams.shape
        # for eachParam in range(allParams.shape[0]):
        #     print allParams[eachParam].shape
        #self.params = None
        for eachParam in range(allParams.shape[0]):
            self.params[eachParam].set_value(allParams[eachParam])
        print 'loaded the saved convnet classifier classifier parameters'
        pass

    def predictThisModel(self):
        accuracyList =[]
        for i in range(100):
            randomTestLabelIndices = self.matrifyMyData(justLabels=True)
            randomTestData = self.dataMatTest[randomTestLabelIndices]
            randomTestLabels = self.labelMatTest[randomTestLabelIndices]
            randomLabels = np.argmax(randomTestLabels, axis=1)
            for i in range(5):
                print "labelCount: ",i,"=", np.where(randomLabels == i)[0].shape[0]
            currentAccuracy = np.mean(np.argmax(randomTestLabels, axis=1) == self.predict(randomTestData)) * 100
            print currentAccuracy, '% accuracy'
            accuracyList.append(currentAccuracy)
        pass
        avgAccuracy = sum(accuracyList)/len(accuracyList)
        print 'avgAccuracy: ', avgAccuracy, '%'
    def writeFirstLayerToDisk(self, imgArray):
        print imgArray.shape
        convOut1 = self.getFirstLayerOutput(imgArray)
        print convOut1.shape
        cv2.imwrite(curDirName+"/vis/0.jpg", convOut1[0, 0,:,:].reshape(204, 204))
        cv2.imwrite(curDirName+"/vis/1.jpg", convOut1[0, 1,:,:].reshape(204, 204))
        cv2.imwrite(curDirName+"/vis/2.jpg", convOut1[0, 2,:,:].reshape(204, 204))
        cv2.imwrite(curDirName+"/vis/3.jpg", convOut1[0, 3,:,:].reshape(204, 204))
        cv2.imwrite(curDirName+"/vis/4.jpg", convOut1[0, 4,:,:].reshape(204, 204))
        cv2.imwrite(curDirName+"/vis/5.jpg", convOut1[0, 5,:,:].reshape(204, 204))
        cv2.imwrite(curDirName+"/vis/6.jpg", convOut1[0, 6,:,:].reshape(204, 204))
        cv2.imwrite(curDirName+"/vis/7.jpg", convOut1[0, 7,:,:].reshape(204, 204))
        cv2.imwrite(curDirName+"/vis/8.jpg", convOut1[0, 8,:,:].reshape(204, 204))
        cv2.imwrite(curDirName+"/vis/9.jpg", convOut1[0, 9,:,:].reshape(204, 204))
        cv2.imwrite(curDirName+"/vis/10.jpg", convOut1[0, 10,:,:].reshape(204, 204))
        cv2.imwrite(curDirName+"/vis/11.jpg", convOut1[0, 11,:,:].reshape(204, 204))
        cv2.imwrite(curDirName+"/vis/12.jpg", convOut1[0, 12,:,:].reshape(204, 204))
        cv2.imwrite(curDirName+"/vis/13.jpg", convOut1[0, 13,:,:].reshape(204, 204))
        cv2.imwrite(curDirName+"/vis/14.jpg", convOut1[0, 14,:,:].reshape(204, 204))
        cv2.imwrite(curDirName+"/vis/15.jpg", convOut1[0, 15,:,:].reshape(204, 204))
        cv2.imwrite(curDirName+"/vis/16.jpg", convOut1[0, 16,:,:].reshape(204, 204))
        cv2.imwrite(curDirName+"/vis/17.jpg", convOut1[0, 17,:,:].reshape(204, 204))
        cv2.imwrite(curDirName+"/vis/18.jpg", convOut1[0, 18,:,:].reshape(204, 204))
        cv2.imwrite(curDirName+"/vis/19.jpg", convOut1[0, 19,:,:].reshape(204, 204))
        exit()

    def predictThisImage(self, imgArray=None):
        #imgArray = self.getMeanNormalizedData(imgArray, 'TEST')
        return self.predict(imgArray)
    def predictThisImageWithProbability(self, imgArray = None):
        #imgArray = self.getMeanNormalizedData(imgArray, 'TEST')
        #imgArray = imgArray.astype('float64')
        return self.predictProb(imgArray)
    
    def getMeanNormalizedData(self, data, mode):
        self.testMean = np.load('/home/jason/Desktop/Robotics/Dataset/lmdb generator/data_lmdb.npy')
        self.testMean = np.reshape(self.testMean, (1, 3, self.imgSize, self.imgSize))
        # print 'subtracting mean'
        if(mode == "TRAIN"):
            data = data.astype('float64') - self.trainMean
        elif(mode == "TEST"):
            data = data.astype('float64') - self.testMean
        data = data/float(255.0)
        return data