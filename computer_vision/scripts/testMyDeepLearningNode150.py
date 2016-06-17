import os
from os.path import expanduser
import traceback

curDirName = os.path.dirname(os.path.abspath(__file__))
from DeepLearningNode import DeepLearningNode
import glob

import random
import numpy as np
import time
from collections import Counter

import os
from os.path import expanduser

# initialize caffe and lmdb loading module
os.chdir(expanduser('~')+'/Programs/caffe/')
import caffe

from caffe.proto import caffe_pb2

import lmdb
import time
import sys
import traceback
import cPickle
from time import gmtime, strftime
import cv2

def matrifyMyData(labelMatTrain):    
    uniqueClasses = np.unique(labelMatTrain)
    nClasses = 2
    # print "nClasses ",nClasses, uniqueClasses
    dLabel = np.eye(nClasses)
    nTrainLabel = []
    try:
        totalTrainSamples = labelMatTrain.shape[0]
        for i in range(totalTrainSamples):
            nTrainLabel.append(dLabel[labelMatTrain[i][0]])
        nTrainLabel = np.array(nTrainLabel)
        labelMatTrain = nTrainLabel
    except:
        print labelMatTrain.shape
        traceback.print_exc(file=sys.stdout)

    return labelMatTrain

LabelDict = {}
LabelDict["Non-Object"] = 0
LabelDict["Object"] = 1
ReverseLabelDict = {}

ReverseLabelDict[0] = "Non-Object"
ReverseLabelDict[1] = "Object"

def returnLabelName(num_id):
    return ReverseLabelDict[num_id]
    pass
def processIt(orig, pred):
    origList = orig.tolist()
    predList = pred.tolist()
    origProcessed = []
    predProcessed = []
    for each in origList:
        origProcessed.append(ReverseLabelDict[each])
    for each in predList:
        predProcessed.append(ReverseLabelDict[each])
    return origProcessed, predProcessed

if __name__ == "__main__":
    start_time = time.time()
    print np.version.version
    print 'now testing the model'

    np.set_printoptions(precision=4)

    print 'program arguments: imgSize = ', sys.argv[1]
    imgSize = int(sys.argv[1])

    modelList = [curDirName+"/c0_9_0.0491234758115_TempModel_DeepLearningNode_150.npz", curDirName+"/c1_9_0.0848671159493_TempModel_DeepLearningNode_150.npz", curDirName+"/c2_9_0.0503356325916_TempModel_DeepLearningNode_150.npz"]
    
    
    # testFile = '/home/ganymede/Datasets/spherical imageset1/raw_images/'+str(imgSize)+' resize/data_lmdb_'+str(imgSize)+'/'
    
    dropout_params = {}
    dropout_params['conv'] = 0.5
    dropout_params['fc'] = 0.5

    for modelToLoad in modelList:
        fname = modelToLoad.split("/")[::-1][0]
        fp = open(curDirName+"/"+str(fname)+'_classification.txt','w')
        print fname
        crossvalidid = int(fname.split("_")[0][1])
        testFile = '/home/ganymede/Datasets/cross validation/'+str(crossvalidid)+'/test_lmdb_'+str(imgSize)+'/'
        print crossvalidid
        print testFile
        print os.path.exists(testFile)
      
    
        mymodel = DeepLearningNode(loadData = False, imgSize = imgSize, crossvalidid = crossvalidid, dropout_params = dropout_params, mode='Test', modelToLoad = modelToLoad)
        
        mini_batch_size = 128

        lmdb_env = lmdb.open(testFile)
        lmdb_txn = lmdb_env.begin()
        lmdb_cursor = lmdb_txn.cursor()
        datum = caffe_pb2.Datum()
        dataList = []
        labelList = []
        totalImgs = 0
        correct = 0
        for key, value in lmdb_cursor:
            
            datum.ParseFromString(value)
            label = datum.label
            data = caffe.io.datum_to_array(datum)
            dataList.append(data)
            labelList.append(label)
            if(len(labelList)==mini_batch_size):
                totalImgs += mini_batch_size
                dataMatTrain = np.array(dataList)
                labelMatTrain = np.array(labelList)
                # print dataMatTrain.shape, labelMatTrain.shape
                totalTrainSamples = dataMatTrain.shape[0]
                labelMatTrain = labelMatTrain.reshape((mini_batch_size, 1))
                # print dataMatTrain.shape, labelMatTrain.shape
                labelMatTrain = matrifyMyData(labelMatTrain)
                # print dataMatTrain.shape, labelMatTrain.shape
                trX, trY = dataMatTrain, labelMatTrain
                trX = mymodel.getMeanNormalizedData(trX, 'TEST')
                
                vals = mymodel.predictProb(trX)
                
                pred = np.argmax(vals, axis=1)
                orig = np.argmax(trY, axis=1)
                correct += np.sum(pred==orig)
                orig, pred = processIt(orig, pred)
                for each in zip(orig, pred):
                    fp.write("orig: "+each[0]+", pred: "+each[1]+"\n")
                dataList = []
                labelList = []
                if(totalImgs %1000==0):
                    print 'processed', totalImgs
        accuracy = 100.0*(float(correct)/totalImgs)
        print fname, 'accuracy: ', accuracy,'%'
        fp.close()