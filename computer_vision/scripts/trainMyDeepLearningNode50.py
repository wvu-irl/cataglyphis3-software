import os
from os.path import expanduser
import cv2
curDirName = os.path.dirname(os.path.abspath(__file__))
from DeepLearningNode import DeepLearningNode
import time
import sys
if __name__ == "__main__":    
    # try real data
    start_time = time.time()
    print 'starting training at ', start_time
    dropout_params = {}
    dropout_params['conv'] = 0.5
    dropout_params['fc'] = 0.5
    print 'program arguments: imgSize = ', sys.argv[1]
    imgSize = int(sys.argv[1])

    for i in range(3):
        crossvalidid = i
        mymodel = DeepLearningNode(loadData = True, imgSize=imgSize, crossvalidid = crossvalidid, mode= 'Train', randomData=False, dropout_params = dropout_params, modelToLoad=None, total_epochs=10)
        # mymodel.trainThisModel()