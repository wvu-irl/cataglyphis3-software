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
from DeepFishNet import DeepFishNet

class ClassifierService:

	def __init__(self, myClassifier, meanData, imgSize, cvModulePath):
		# myClassifier, meanData, imgSize, cvModulePath
		print 'ClassifierService constructor'
		time.sleep(1)
		self.deepFishNet = myClassifier
		self.meanData = meanData
		self.imgSize = imgSize
		self.cvModulePath = cvModulePath
		print 'successfully loaded the classifier'
		time.sleep(1)

	def getMeanNormalizedData(self, img):
		# print img.shape
		img = cv2.resize(img, (self.imgSize, self.imgSize))
		# print img.shape
		imgB, imgG, imgR = cv2.split(img)
		# print imgB.shape
		# print imgG.shape
		# print imgR.shape
		tensorImg = np.zeros((1, 3, self.imgSize, self.imgSize))
		tensorImg[0, 0, :, :] = imgB
		tensorImg[0, 1, :, :] = imgG
		tensorImg[0, 2, :, :] = imgR
		tensorImg = tensorImg.astype('float64') - self.meanData
		tensorImg = tensorImg/float(255.0)
		# print 'returning shape', tensorImg.shape
		return tensorImg

	def readImgs(self, numBlobs):
		# numBlobs = 48, read imgs from 0-47
		imgList = []
		for eachIndex in range(numBlobs):
			curImgPath = self.cvModulePath+'/data/blobs/blob'+str(eachIndex)+'.jpg'
			print 'reading ', curImgPath
			img = cv2.imread(curImgPath, 1)
			meanNormalizedImg = self.getMeanNormalizedData(img)
			imgList.append(meanNormalizedImg)
		imgList = np.array(imgList)
		# print 'imgList shape: ',imgList.shape
		imgList = np.reshape(imgList, (numBlobs, 3, self.imgSize, self.imgSize))
		# print 'imgList shape: ',imgList.shape
		return imgList

	def handle_classification_request(self, incomingMessage):
		startTime = time.clock()
		numBlobs = int(incomingMessage.numBlobs)
		positiveConfidenceList = []
		try:
			# load images, subtract mean and normalize
			tensorBlobImgs = self.readImgs(numBlobs)			
		except:
			print "Exception in reading the feature vectors and subtracting mean"
			traceback.print_exc(file=sys.stdout)
			os.system("rosnode kill /classify_feature_vector_server")
			pass
		try:
			# classify
			predictedProbabilities = self.deepFishNet.predictProb(tensorBlobImgs)
			# predictedProbabilities[:,0] contains non-object probabilities
			# predictedProbabilities[:,1] contains object probabilities

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
	
	# read the mean data
	meanDataPath = cvModulePath+"/data/mean_file/"+'data_lmdb.npy'
	meanData = np.load(meanDataPath)
	meanData = np.reshape(meanData, (1, 3, imgSize, imgSize))

	# read the classifier
	classifierPath = cvModulePath+'/data/classifier/554_convnet_150imgsize.npz'

	# set dropout parameters for better performance
	dropout_params = {}
	dropout_params['conv'] = 0.4
	dropout_params['fc'] = 0.4
	
	# initialize DeepFishNet
	myClassifier = DeepFishNet(mode='Test', modelToLoad = classifierPath, dropout_params = dropout_params)

	# initialize classifier service
	cService = ClassifierService(myClassifier, meanData, imgSize, cvModulePath)
	cService.startService()
	pass