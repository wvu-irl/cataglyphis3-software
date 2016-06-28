#!/usr/bin/python
'''
AUTHOR: Rahul Kavi
Feature Vector generator/classifier for FishEye images for CATAGLYPHIS
'''
from __future__ import division

import sys
import os
import traceback
import time

import numpy as np
import cv2
import rospy
import rospkg

from computer_vision.srv import *
from std_msgs.msg import String
# from DeepFishNet import DeepFishNet
from DeepFishNet150 import DeepFishNet150
from DeepFishNet50 import DeepFishNet50

class ClassifierService:

	def __init__(self, classifierDict):
		'''
		ClassifierService constructor takes classifier dictionary as parameters
			# per imgResolution of the image, classifierDict has classifier, meanData, classifier module path
		'''
		print 'ClassifierService constructor'
		# get CV module path
		self.classifierDict = classifierDict
		time.sleep(1)
		imgSize = 50
		self.cvModulePath = self.classifierDict[imgSize]['modulePath']
		print 'successfully loaded the classifier'
		time.sleep(1)

	def getMeanNormalizedData(self, img, imgSize):
		imgB, imgG, imgR = cv2.split(img)
		# print imgB.shape
		# print imgG.shape
		# print imgR.shape
		tensorImg = np.zeros((1, 3, imgSize, imgSize))
		tensorImg[0, 0, :, :] = imgB
		tensorImg[0, 1, :, :] = imgG
		tensorImg[0, 2, :, :] = imgR

		tensorImg = tensorImg.astype('float64') - classifierDict[imgSize]['mean']
		tensorImg = tensorImg/float(255.0)
		# print 'returning shape', tensorImg.shape
		return tensorImg

	def readImgs(self, numBlobs, imgSize):
		# numBlobs = 48, read imgs from 0-47
		imgList = []
		for eachIndex in range(numBlobs):
			curImgPath = self.cvModulePath+'/data/blobs/blob'+str(eachIndex)+'.jpg'
			print 'reading ', curImgPath
			img = cv2.imread(curImgPath, 1)
			img = cv2.resize(img, (imgSize, imgSize))
			meanNormalizedImg = self.getMeanNormalizedData(img, imgSize)
			imgList.append(meanNormalizedImg)
		imgList = np.array(imgList)
		# print 'imgList shape: ',imgList.shape
		imgList = np.reshape(imgList, (numBlobs, 3, imgSize, imgSize))
		# print 'imgList shape: ',imgList.shape
		return imgList

	def handle_classification_request(self, incomingMessage):
		startTime = time.clock()
		numBlobs = int(incomingMessage.numBlobs)
		imgSize = int(incomingMessage.imgSize)
		positiveConfidenceList = []
		try:
			# load images, subtract mean and normalize
			tensorBlobImgs = self.readImgs(numBlobs, imgSize)
		except:
			print "Exception in reading the feature vectors and subtracting mean"
			traceback.print_exc(file=sys.stdout)
			os.system("rosnode kill /classify_feature_vector_server")
			pass
		try:
			print tensorBlobImgs.shape
			# classify
			predictedProbabilities = self.classifierDict[imgSize]['classifier'].predictProb(tensorBlobImgs)
			# retreive only object probabilities
			predictedProbabilities = predictedProbabilities[:,1]
			# convert to list and for returning it to the client
			positiveConfidenceList = predictedProbabilities.tolist()
			pass
		except:
			print 'obtaining probabilities from extracted probabilities'
			traceback.print_exc(file=sys.stdout)
			os.system("rosnode kill /classify_feature_vector_server")
			pass

		endTime = time.clock()
		print 'time taken totally for classification: ', endTime -startTime, 'seconds'
		return ImageProbabilitiesResponse(positiveConfidenceList)

	def startService(self):
		rospy.init_node('classify_feature_vector_server')
		try:
			serviceObject = rospy.Service('classify_feature_vector_service', ImageProbabilities, self.handle_classification_request)
			print 'running classification server'
			print 'waiting for requests'
			rospy.spin()
		except:
			print 'error starting classification service'
			traceback.print_exc(file=sys.stdout)
			os.system("rosnode kill /classify_feature_vector_server")

if __name__ == "__main__":
	rospack = rospkg.RosPack()

	# get CV module path
	cvModulePath = rospack.get_path('computer_vision')

	# imgSize
	imgSize = 150
	
	# dictionary to store classifier parameters
	classifierDict = {}

	# read the mean data
	meanData150Path = cvModulePath+"/data/mean_file/"+str(imgSize)+'_x_'+str(imgSize)+'_mean/'+'data_lmdb.npy'
	# meanData150Path = cvModulePath+ "/data/mean_file/50_x_50_mean/allData_150_lmdb.npy"
	meanData150 = np.load(meanData150Path)
	meanData150 = np.reshape(meanData150, (1, 3, imgSize, imgSize))
	
	# read the classifier
	# classifier150Path = cvModulePath+'/data/classifier/DeepFishNet'+str(imgSize)+'/DeepFishNet'+str(imgSize)+'.npz'
	classifier150Path = cvModulePath+'/data/classifier/DeepFishNet'+str(imgSize)+'/best_9epoch_'+str(imgSize)+'.npz'

	# set dropout parameters for better performance
	dropoutParams150 = {}
	dropoutParams150['conv'] = 0.5
	dropoutParams150['fc'] = 0.5
	
	# initialize DeepFishNet150
	# myClassifier = DeepFishNet(mode='Test', modelToLoad = classifier150Path, dropout_params = dropoutParams150)
	myClassifier150 = DeepFishNet150(loadData = False, imgSize = imgSize, crossvalidid = 0, dropout_params = dropoutParams150, mode='Test', modelToLoad = classifier150Path)
	
	# store 150 x 150 classifier details
	classifierDict[imgSize] = {}
	classifierDict[imgSize]['classifier'] = myClassifier150
	classifierDict[imgSize]['mean'] = meanData150
	classifierDict[imgSize]['modulePath'] = cvModulePath
	
	# imgSize
	imgSize = 50
	
	# read the mean data
	# meanData50Path = cvModulePath+"/data/mean_file/"+str(imgSize)+'_x_'+str(imgSize)+'_mean/'+'data_lmdb.npy'
	meanData50Path = cvModulePath+ "/data/mean_file/50_x_50_mean/allData_50_lmdb.npy"
	meanData50 = np.load(meanData50Path)
	meanData50 = np.reshape(meanData50, (1, 3, imgSize, imgSize))
	
	# read the classifier
	# classifier50Path = cvModulePath+'/data/classifier/DeepFishNet'+str(imgSize)+'/DeepFishNet'+str(imgSize)+'.npz'
	classifier50Path = cvModulePath+'/data/classifier/DeepFishNet'+str(imgSize)+'/best_9epoch_'+str(imgSize)+'.npz'
	
	# set dropout parameters for better performance
	dropoutParams50 = {}
	dropoutParams50['conv'] = 0.5
	dropoutParams50['fc'] = 0.5
	
	# initialize DeepFishNet50
	myClassifier50 = DeepFishNet50(loadData = False, imgSize = imgSize, crossvalidid = 0, dropout_params = dropoutParams50, mode='Test', modelToLoad = classifier50Path)

	# store 50 x 50 classifier details
	classifierDict[imgSize] = {}
	classifierDict[imgSize]['classifier'] = myClassifier50
	classifierDict[imgSize]['mean'] = meanData50
	classifierDict[imgSize]['modulePath'] = cvModulePath

	# initialize classifier service
	cService = ClassifierService(classifierDict)
	cService.startService()
	pass