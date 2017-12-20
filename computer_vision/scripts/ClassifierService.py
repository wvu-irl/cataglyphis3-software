#!/usr/bin/python
'''
/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2016, WVU Interactive Robotics Laboratory
*                       https://web.statler.wvu.edu/~irl/
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
'''
'''
AUTHOR: Rahul Kavi
EDITED BY: Jared Strader
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
		classifierType = 0 #initialize to precached (0 = precached, 1 = rock, 2 = hard)
		self.cvModulePath = self.classifierDict[classifierType]['modulePath']
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

		tensorImg = tensorImg.astype('float64') - classifierDict[classifierType]['mean']
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
		classifierType = int(incomingMessage.classifierType)
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
			predictedProbabilities = self.classifierDict[classifierType]['classifier'].predictProb(tensorBlobImgs)
			# retreive only object probabilities
			predictedProbabilities = predictedProbabilities[:,1]
			# convert to list and for returning it to the client
			positiveConfidenceList = predictedProbabilities.tolist()
			# print positiveConfidenceList
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

	# dictionary to store classifier parameters
	classifierDict = {}

	# # classifier version
	# classifierVersion = 'CACH_V1'
	# classifierSampleType = 'CACH'
	# classifierType = 0

	# # imgSize
	# imgSize = 50

	# # read the mean data
	# meanData50PathCach = cvModulePath+'/data/classifier/DeepFishNet'+str(imgSize)+'/'+classifierVersion+"/allData_50_lmdb_"+classifierSampleType+".npy"
	# meanData50Cach = np.load(meanData50PathCach)
	# meanData50Cach = np.reshape(meanData50Cach, (1, 3, imgSize, imgSize))

	# # read the classifier
	# classifier50PathCach = cvModulePath+'/data/classifier/DeepFishNet'+str(imgSize)+'/'+classifierVersion+'/best_9epoch_50_'+classifierSampleType+'.npz'

	# # set dropout parameters for better performance
	# dropoutParams50Cach = {}
	# dropoutParams50Cach['conv'] = 0.5
	# dropoutParams50Cach['fc'] = 0.5

	# # initialize DeepFishNet50Cach
	# myClassifier50Cach = DeepFishNet50(loadData = False, imgSize = imgSize, crossvalidid = 0, dropout_params = dropoutParams50Cach, mode='Test', modelToLoad = classifier50PathCach)

	# # store 50 x 50 classifier details
	# classifierDict[classifierType] = {}
	# classifierDict[classifierType]['classifier'] = myClassifier50Cach
	# classifierDict[classifierType]['mean'] = meanData50Cach
	# classifierDict[classifierType]['modulePath'] = cvModulePath

	# # classifier version
	# classifierVersion = 'HARD_V1'
	# classifierSampleType = 'HARD'
	# classifierType = 1

	# # imgSize
	# imgSize = 50

	# # read the mean data
	# meanData50PathHard = cvModulePath+'/data/classifier/DeepFishNet'+str(imgSize)+'/'+classifierVersion+"/allData_50_lmdb_"+classifierSampleType+".npy"
	# meanData50Hard = np.load(meanData50PathHard)
	# meanData50Hard = np.reshape(meanData50Hard, (1, 3, imgSize, imgSize))

	# # read the classifier
	# classifier50PathHard = cvModulePath+'/data/classifier/DeepFishNet'+str(imgSize)+'/'+classifierVersion+'/best_9epoch_50_'+classifierSampleType+'.npz'

	# # set dropout parameters for better performance
	# dropoutParams50Hard = {}
	# dropoutParams50Hard['conv'] = 0.5
	# dropoutParams50Hard['fc'] = 0.5

	# # initialize DeepFishNet50Hard
	# myClassifier50Hard = DeepFishNet50(loadData = False, imgSize = imgSize, crossvalidid = 0, dropout_params = dropoutParams50Hard, mode='Test', modelToLoad = classifier50PathHard)

	# # store 50 x 50 classifier details
	# classifierDict[classifierType] = {}
	# classifierDict[classifierType]['classifier'] = myClassifier50Hard
	# classifierDict[classifierType]['mean'] = meanData50Hard
	# classifierDict[classifierType]['modulePath'] = cvModulePath

	# # classifier version
	# classifierVersion = 'ROCK_V1'
	# classifierSampleType = 'ROCK'
	# classifierType = 2

	# # imgSize
	# imgSize = 50

	# # read the mean data
	# meanData50PathRock = cvModulePath+'/data/classifier/DeepFishNet'+str(imgSize)+'/'+classifierVersion+"/allData_50_lmdb_"+classifierSampleType+".npy"
	# meanData50Rock = np.load(meanData50PathRock)
	# meanData50Rock = np.reshape(meanData50Rock, (1, 3, imgSize, imgSize))

	# # read the classifier
	# classifier50PathRock = cvModulePath+'/data/classifier/DeepFishNet'+str(imgSize)+'/'+classifierVersion+'/best_9epoch_50_'+classifierSampleType+'.npz'

	# # set dropout parameters for better performance
	# dropoutParams50Rock = {}
	# dropoutParams50Rock['conv'] = 0.5
	# dropoutParams50Rock['fc'] = 0.5

	# # initialize DeepFishNet50Rock
	# myClassifier50Rock = DeepFishNet50(loadData = False, imgSize = imgSize, crossvalidid = 0, dropout_params = dropoutParams50Rock, mode='Test', modelToLoad = classifier50PathRock)

	# # store 50 x 15 classifier details
	# classifierDict[classifierType] = {}
	# classifierDict[classifierType]['classifier'] = myClassifier50Rock
	# classifierDict[classifierType]['mean'] = meanData50Rock
	# classifierDict[classifierType]['modulePath'] = cvModulePath

	# classifier version
	classifierVersion = 'ALL_V1'
	classifierSampleType = 'ALL'
	classifierType = 0

	# imgSize
	imgSize = 50

	# read the mean data
	meanData50PathAll = cvModulePath+'/data/classifier/DeepFishNet'+str(imgSize)+'/'+classifierVersion+"/allData_50_lmdb_"+classifierSampleType+".npy"
	meanData50All = np.load(meanData50PathAll)
	meanData50All = np.reshape(meanData50All, (1, 3, imgSize, imgSize))

	# read the classifier
	classifier50PathAll = cvModulePath+'/data/classifier/DeepFishNet'+str(imgSize)+'/'+classifierVersion+'/best_9epoch_50_'+classifierSampleType+'.npz'

	# set dropout parameters for better performance
	dropoutParams50All = {}
	dropoutParams50All['conv'] = 0.5
	dropoutParams50All['fc'] = 0.5

	# initialize DeepFishNet50Rock
	myClassifier50All = DeepFishNet50(loadData = False, imgSize = imgSize, crossvalidid = 0, dropout_params = dropoutParams50All, mode='Test', modelToLoad = classifier50PathAll)

	# store 50 x 15 classifier details
	classifierDict[classifierType] = {}
	classifierDict[classifierType]['classifier'] = myClassifier50All
	classifierDict[classifierType]['mean'] = meanData50All
	classifierDict[classifierType]['modulePath'] = cvModulePath

	# initialize classifier service
	cService = ClassifierService(classifierDict)
	cService.startService()
	pass
