#!/usr/bin/python
'''
AUTHOR: Rahul Kavi
Feature Vector generator/classifier for FishEye images for CATAGLYPHIS

Description:
The Deep learning architecture follows the following convention:
    2 Convolutional layers
    2 Fully Connected layers
    1 SoftMax layer
'''
import theano
from theano import tensor as T
from theano.sandbox.rng_mrg import MRG_RandomStreams as RandomStreams
import numpy as np
from theano.tensor.nnet.conv import conv2d
from theano.tensor.signal.downsample import max_pool_2d
from lasagne.init import GlorotUniform
import os
from os.path import expanduser
import cv2
curDirName = os.path.dirname(os.path.abspath(__file__))

# initialize caffe and lmdb loading module
# os.chdir(expanduser('~')+'/Programs/caffe/')
# import caffe

# from caffe.proto import caffe_pb2

# import lmdb
import time
import sys
import traceback
import cPickle
from time import gmtime, strftime

# theano.config.compute_test_value = 'warn' # Use 'warn' to activate this feature

class DeepFishNet50:
    '''
    this class represents a convolutional neural network with 3 convolutional layers
    and 2 fully connected layers connected to 1 layer LSTM cells with a final softmax layer. Predicts 5 classes
    '''
    def __init__(self, imgSize = None, crossvalidid = None,loadData = True, mode= None, modelToLoad=None, randomData=False, dropout_params = None, caffeModelName=None, total_epochs = 10 ):
        '''
            DeepFishNet50 constructor for classifying
            50 x 50 images.
            Initializes variables and the weights associated with the network
        '''

        assert(mode != None)
        
        self.mode = mode
        self.srng = RandomStreams()
        self.imgSize = imgSize
        self.dataMatTrain = None
        self.labelMatTrain = None
        self.dataMatTest = None
        self.labelMatTest = None
        self.dropout_params = dropout_params
        self.total_epochs = total_epochs
        self.crossvalidid = crossvalidid

        self.totalTrainSamples = None
        self.totalTestSamples = None
        self.randomData = None
        self.caffeModelName = caffeModelName
        self.modelToLoad = modelToLoad
        # initialize your model
        self.initializeModel()

        if(mode == 'Train'):
            print 'call train your model'
            self.trainThisModel()
            # save your model for future use
            #self.saveThisModel()
        elif(mode == 'Test'):
            assert(modelToLoad != None)
            self.loadThisModel(modelToLoad)
            pass
        pass

    def moveToCaffeDir(self):
        '''
            util function to move to cur dir to
        '''
        os.chdir(expanduser('~')+'/Programs/caffe/')
        pass

    def matrifyMyData(self):
        '''
            returns labels in one hot form
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

        return

    
    def floatX(self, X):
        '''
            # float casting your input
        '''
        return np.asarray(X, dtype=theano.config.floatX)

    def init_weights(self, shape, weightType = None, typeLayer = None, caffeLayerName = None):
        '''
            initialize your weights
        '''
        if(weightType == 'Xavier' and typeLayer == None):
            W=GlorotUniform()
            weights = W.sample(shape)
            if(self.mode == "Train"):
                if(self.caffeModelName != None and caffeLayerName != None):
                    caffeWeights = self.loadTheseWeights(self.caffeModelName, caffeLayerName)
                    print caffeWeights.shape
                print weights.shape
            print 'returning Xavier weights'
            return theano.shared(self.floatX(weights), borrow=True)
        return theano.shared(self.floatX(np.random.randn(*shape) * 0.01),borrow=True)

    def sigmoid(self, X):
        '''
            # 1 apply non-linear activation function of the given input.
        '''
        return 1.0 / (1.0 + T.exp(-X))

    def rectify(self, X):
        '''
            # 1 apply non-linear activation function of the given input.
            # 2 aparently ReLU is faster than signoid/tanh
        '''
        return T.maximum(X, 0.)
    
    def softmax(self, X):
        '''
            # get your final softmax classifier so that it returns probabilities
        '''
        # make it mathematically easier
        e_x = T.exp(X - X.max(axis=1).dimshuffle(0, 'x'))
        return e_x / e_x.sum(axis=1).dimshuffle(0, 'x')

    
    def dropout(self, X, p=0.):
        '''
            # define dropout function with the given probability of retaining
        '''
        if p > 0:
            retain_prob = 1 - p
            X *= self.srng.binomial(X.shape, p=retain_prob, dtype=theano.config.floatX)
            X /= retain_prob
        return X

    
    def RMSprop(self, costC, paramsC, lr=0.02, rho=0.9, epsilon=1e-6):
        '''
            # your objective function cost minimizing function.
        '''
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

    # 2 convolutional layers
    # flatten the output
    # then pass it to 
    # 2 fully connected layers
    # connect it to softmax layer to get the probabilities

    def model(self, X, w1, w2, w4, w5, w_output, p_drop_conv = 0, p_drop_hidden = 0):
        '''
            # your main model 
            2 convolutional layers
            2 fully connected layers
            1 SoftMax layer
        '''
        # first convolutional layer
        if(self.mode == "Train"):
            l1a = self.rectify(conv2d(X, w1, border_mode='full'))
            l1 = max_pool_2d(l1a, (2, 2), st=None, padding=(0, 0), mode='max')
            l1 = self.dropout(l1, p_drop_conv)
        elif(self.mode == "Test"):
            l1a = self.rectify(conv2d(X, w1*(1-p_drop_conv), border_mode='full'))
            l1 = max_pool_2d(l1a, (2, 2), st=None, padding=(0, 0), mode='max')
        # convOut1 = conv2d(X, w1)
        convOut1 = l1a
        
        # second convolutional layer
        
        if(self.mode == "Train"):
            l2a = self.rectify(conv2d(l1, w2))
            l2 = max_pool_2d(l2a, (2, 2), st=None, padding=(0, 0), mode='max')
            l2 = self.dropout(l2, p_drop_conv)
        elif(self.mode == "Test"):
            l2a = self.rectify(conv2d(l1, w2*(1-p_drop_conv)))
            l2 = max_pool_2d(l2a, (2, 2), st=None, padding=(0, 0), mode='max')


        # # flatten the output
        l3 = T.flatten(l2, outdim=2)
        
        # 1st fully connected layer
        if(self.mode == "Train"):
            l4 = self.rectify(T.dot(l3, w4))
            l4 = self.dropout(l4, p_drop_hidden)
        elif(self.mode == "Test"):
            l4 = self.rectify(T.dot(l3, w4 * (1-p_drop_hidden)))

        # 2nd fully connected layer
        if(self.mode == "Train"):
            l5 = self.rectify(T.dot(l4, w5))
            l5 = self.dropout(l5, p_drop_hidden)
        elif(self.mode == "Test"):
            l5 = self.rectify(T.dot(l4, w5 * (1-p_drop_hidden)))

        # connected the above output to softmax layer
        pyx = self.softmax(T.dot(l5, w_output))
        
        return l1, l2, l3, l4, l5, pyx, convOut1

    def getL1Norm(self, params, scaleForm=0.0001):
        '''
            get L1 normalization on the weights.
            performing regularization using L1 reduces the size of the weights and makes them sparse
        '''
        tsum = 0
        for eachParam in params:
            tsum += abs(eachParam).sum()
        return tsum*scaleForm
        pass

    def getL2Norm(self, params, scaleForm=0.0001):
        '''
            get L2 normalization on the weights.
            performing regularization using L2 gives rise to uniqe solution for the weights
        '''
        tsum = 0
        for eachParam in params:
            tsum += T.sqrt(eachParam ** 2).sum()
        return tsum*scaleForm

    def getL2NormSquare(self, params, scaleForm=0.0001):
        '''
            get L2 (square) normalization  on the weights.
            less computationally expensive than pure L2 normalization
        '''
        tsum = 0
        for eachParam in params:
            tsum += abs(eachParam ** 2).sum()
        return tsum*scaleForm
    
    def initializeModel(self):
        '''
            define your deep learning model
        '''
        print 'defining model'

        X = T.ftensor4()
        Y = T.fmatrix()

        #initialize your weghts, kernels
        # format n kernels, n channels, kernel_w x kernel_h
        # 20 kernels on gray scale image with 5 x 5 sized kernel
        w1 = self.init_weights((20, 3, 5, 5), weightType = 'Xavier', caffeLayerName = 'conv1')

        # 50 20-channel 5 x 5 sized kernel
        w2 = self.init_weights((50, 20, 5, 5), weightType = 'Xavier', caffeLayerName = 'conv2')

        # flatten the inputs and pass to fully connected layer
        w4 = self.init_weights((7200, 1000), weightType = 'Xavier')

        # flatten the inputs and pass to fully connected layer
        w5 = self.init_weights((1000, 500), weightType = 'Xavier')

                
        # flatten the inputs and pass to fully connected layer
        w_output = self.init_weights((500, 2), weightType = 'Xavier')

        # define your deep model
        if(self.dropout_params == None):
            # if there is no default dropout params mentioned, just set them manually
            self.dropout_params = {}
            self.dropout_params['conv'] = 0.1
            self.dropout_params['fc'] = 0.2
        print 'initializing with dropout_params: ', self.dropout_params['conv'], self.dropout_params['fc']
        noise_l1, noise_l2, noise_l3, noise_l4, noise_l5, noise_py_x, convOut1 = self.model(X, w1, w2, w4, w5, w_output, p_drop_conv = self.dropout_params['conv'], p_drop_hidden = self.dropout_params['fc'])
     
        # get your label from the predicted probabilties
        y_x = T.argmax(noise_py_x, axis=1)
        # y_x = noise_py_x >= 0.5

        self.learning_rate = 0.0001

        self.params = [w1, w2, w4, w5, w_output]

        
        L1_norm = self.getL1Norm(self.params)
        L2_norm = self.getL2Norm(self.params)

        # pd = np.array(self.params)
        # mean cross entropy with L2 regularization
        self.cost = T.mean(T.nnet.categorical_crossentropy(noise_py_x, Y))

        self.paramUpdates = self.RMSprop(self.cost, self.params, lr = self.learning_rate)
        #self.paramUpdates = self.MomentumOptimizer(self.cost, self.params, lr = self.learning_rate)

        if(self.modelToLoad != None):
            self.loadThisModel(self.modelToLoad)

        # self.cost = T.mean((T.nnet.binary_crossentropy(noise_py_x, Y)))

        print 'compiling functions'
        print 'current learning rate: ', self.learning_rate
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
        print 'compiled the functions, ended at ', strftime("%Y-%m-%d %H:%M:%S")
        print 'time takent compile the functions: ', end_compilation_time - start_compilation_time

    def trainThisModel(self):
        '''
            iterate through the data
            train the classifier
        '''
        print 'training the model'
        # self.total_epochs = 10
        self.mini_batch_size = 32

        still_looping = True
        for each_epoch in range(self.total_epochs):
            if(still_looping == False):
                print 'exiting each_epoch loop'
                break

            print '--'*5
            print 'epoch: ', each_epoch
            print '--'*5
            iterId = 0
            costList = []
            self.cost = 0

            # trainFile = '/home/ganymede/Datasets/cross validation/'+str(self.crossvalidid)+'/train_lmdb_'+str(self.imgSize)+'/'
            # trainMean = np.load('/home/ganymede/Datasets/cross validation/'+str(self.crossvalidid)+'/train_lmdb_'+str(self.imgSize)+'.npy')

            # print trainMean.shape

            # self.trainMean = np.reshape(trainMean, (1, 3, self.imgSize, self.imgSize))

            # print trainMean.shape
            # print trainFile

            lmdb_env = lmdb.open(trainFile)
            lmdb_txn = lmdb_env.begin()
            lmdb_cursor = lmdb_txn.cursor()
            datum = caffe_pb2.Datum()
            dataList = []
            labelList = []
            for key, value in lmdb_cursor:
                datum.ParseFromString(value)
                label = datum.label
                data = caffe.io.datum_to_array(datum)
                dataList.append(data)
                labelList.append(label)
                
                if(len(dataList)==self.mini_batch_size and len(labelList) == self.mini_batch_size):

                    self.dataMatTrain = np.array(dataList)
                    self.labelMatTrain = np.array(labelList)
                    self.totalTrainSamples = self.dataMatTrain.shape[0]
                    # print self.dataMatTrain.shape, self.labelMatTrain.shape
                    self.labelMatTrain = self.labelMatTrain.reshape((self.mini_batch_size, 1))
                    # print self.dataMatTrain.shape, self.labelMatTrain.shape
                    self.matrifyMyData()
                    # print self.dataMatTrain.shape, self.labelMatTrain.shape
                    trX, trY = self.dataMatTrain, self.labelMatTrain
                    trX = self.getMeanNormalizedData(trX, 'TRAIN')
                    ccost = self.train(trX, trY)
                    # print trX.shape, trY.shape
                    # exit()
                    # imgTG = trX[0,:,:,:]
                    # imgTGLabel = trY[0,:]
                    # print imgTG.shape
                    # imgB, imgG, imgR = imgTG[0,:,:],imgTG[1,:,:],imgTG[2,:,:]
                    # imgTG = cv2.merge((imgB, imgG, imgR))
                    # print imgTG.shape, imgTGLabel
                    # cv2.imshow("imgTG", imgTG)
                    # cv2.waitKey(0)

                    # print 'mean: ',np.mean(trX,axis=0)
                    predVals = self.predict(trX)
                    # for each in zip(predVals, trY):
                    #     print each[0], each[1]
                    # print '--'
                    # print 100.0*np.sum(predVals == trY)/float(predVals.shape[0])
                    if(iterId%100==0):
                        print 'epoch: ', each_epoch, 'iter: '+str(iterId)+', cost: ', ccost
                        costList.append(ccost)
                        # print predVals.shape, self.labelMatTrain.shape, trY.shape
                        # for itt1, itt2 in zip(trY, predVals):
                        #     print itt1, itt2
                        pd2 = np.argmax(trY, axis=1)
                        print '--'
                        # print str(100.0*np.sum(predVals == pd2)/float(predVals.shape[0]))+'%'
                        # print str(100.0 * np.sum(predVals == trY)/float(predVals.shape[0]))+'%'
                        pass
                    pass
                    iterId += 1
                    dataList = []
                    labelList = []
                
            avg_cost = sum(costList)/float(len(costList))
            self.saveThisModel('c'+str(self.crossvalidid)+"_"+str(each_epoch)+'_'+str(avg_cost)+'_TempModel_DeepLearningNode_'+str(self.imgSize)+'.npz')
            
        
    def saveThisModel(self, fileName = None):
        '''
            get the variables from the classifier model
            save the model to disk
        '''
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
        # self.moveToCaffeDir()
        pass
    
    def loadThisModel(self, modelToLoad):
        '''
            take the path of the classifier
            load the classifier model
            assign the variables to initialized model
        '''
        print 'loading this model'
        print modelToLoad
        print os.path.exists(modelToLoad) == True
        params = np.load(modelToLoad)
        allParams = params['params']
        print type(allParams)
        print allParams.shape
        for eachParam in range(allParams.shape[0]):
            print allParams[eachParam].shape
        #self.params = None
        for eachParam in range(allParams.shape[0]):
            self.params[eachParam].set_value(allParams[eachParam])
        print 'loaded the saved convnet classifier params'
        # self.moveToCaffeDir()
        pass

    def writeFirstLayerToDisk(self, imgArray):
        '''
            take image
            convolve the image with first layer filters
            write filters to disk
        '''
        # print imgArray.shape
        convOut1 = self.getFirstLayerOutput(imgArray)
        # print convOut1.shape
        # cv2.imwrite(curDirName+"/vis/0.jpg", convOut1[0, 0,:,:].reshape(204, 204))
        # cv2.imwrite(curDirName+"/vis/1.jpg", convOut1[0, 1,:,:].reshape(204, 204))
        pass

    def predictThisImage(self, imgArray=None, meanSubtracted = True):
        '''
            1 take image
            get mean normalized image
            return class of the image
        '''
        if(meanSubtracted == False):
            imgArray = self.getMeanNormalizedData(imgArray, 'Test')
        return self.predict(imgArray)
    
    def predictThisImageWithProbability(self, imgArray = None, meanSubtracted = True):
        '''
            1 take image
            get mean normalized image
            return probabilties of the image being an object, non object
        '''
        if(meanSubtracted == False):
            imgArray = self.getMeanNormalizedData(imgArray, 'Test')
        return self.predictProb(imgArray)
    
    def getMeanNormalizedData(self, data, mode):
        '''
            # 1 take image
            # 2 reshape it
            # 3 subtract mean image
            # 4 divide by 255
            # 5 return image
        '''
        self.testMean = np.load('/home/ganymede/Datasets/cross validation/'+str(self.crossvalidid)+'/test_lmdb_'+str(self.imgSize)+'.npy')
        self.testMean = np.reshape(self.testMean, (1, 3, self.imgSize, self.imgSize))
        data = data.astype('float32')
        # print 'subtracting mean'
        if(mode == "Train"):
            data -= self.trainMean
        elif(mode == "Test"):
            data -= self.testMean
        data = data/float(255.0)
        return data