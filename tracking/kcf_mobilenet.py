# USAGE (for real case application):
#
# 	python human_recog.py --mode 2 --target cpu
# 	python human_recog.py -m 2 -t cpu

# import the necessary packages
from imutils.video import FPS
import argparse
import imagezmq
import socket
import signal
import time
import sys
import numpy as np
import cv2
import dlib

from multiprocessing import Process
from multiprocessing import Queue
from multiprocessing import Value
from pymavlink import mavutil

import sys, os
current_path = os.getcwd()
sys.path.insert(0, current_path)
from config import *
import threading

	

#=======================================================================	
# construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--input", type=str,default=0,
	help="path to the input video file")
ap.add_argument("-o", "--output", type=str,
	help="path to optional output video file")	

args = vars(ap.parse_args())



#=======================================================================
# load our serialized model from disk
print("[INFO] : Loading dnn MobileNet model...")
net = cv2.dnn.readNetFromCaffe(configPath,weightsPath)


# preferable backend to OpenCV
net.setPreferableTarget  (cv2.dnn.DNN_TARGET_CPU)
net.setPreferableBackend (cv2.dnn.DNN_BACKEND_OPENCV)

# the DNN just processed the frame 
dnnWork = 0


#=======================================================================
# Based on the input grab, a reference to the video file or to camera
print("[INFO] : Opening input video file...")
cap = cv2.VideoCapture(args["input"])
fps_cap = int(cap.get(cv2.CAP_PROP_FPS))
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

size = (width, height)

result = cv2.VideoWriter(args["output"], cv2.VideoWriter_fourcc(*'XVID'), 30, (width, height))

#=======================================================================
print("[INFO] : Starting human detection...")

# start the frames per second throughput estimator
fps = FPS().start()

#output save images
path = r"C:\Users\NGHIA\Downloads\anh\\"
index = 0
#=======================================================================
# loop over frames from the video stream

prev_time = 0
while True:
	index += 1
	# grab the next frame
	if not args.get("input", False):
		# read the frame from the camera 
		ret, frame = cap.read()

		if ret == False:
			print ("[Error] It was was impossible to aquire a frame!")
		else:        
			# flips the frame vertically to compensate the camera mount
			frame = cv2.flip(frame,0)   
	else: 
		frame = cap.read()[1]

	# Having a video and we did not grab a frame then we
	# have reached the end of the video
	if frame is None:
		break
	frame = cv2.resize(frame,(750,550))
	# convert the frame from BGR to RGB for dlib
	rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)		

	# check to see if the frame dimensions are not set
	if W is None or H is None:
		# set the frame dimensions
		(H, W) = frame.shape[:2]

	if noFrames % skip_frames == 0:
		dnnWork = 1

		# initialize a new set of detected human
		trackers = []
		confidences = []


		# convert the frame to a blob 
		blob = cv2.dnn.blobFromImage(frame, size=(300, 300), ddepth=cv2.CV_8U)
		# print("First Blob: {}".format(blob.shape))
	
		# send the blob to the network
		net.setInput(blob, scalefactor=1.0/127.5, mean=[127.5, 127.5, 127.5])
	
		# pass the blob through the network and obtain the detections	
		networkOutput = net.forward()

		for detection in networkOutput[0, 0]:
				
			humanClass = int(detection[1])
			if CLASSES[humanClass] != "person":
				continue
		
			confidence = float(detection[2])
		
			# require a minimum confidence to reject fals positive detection
			if confidence > 0.35:
				
				confidences.append(confidence)
				
				# work on the current frame
				#====================================
				left   = int(detection[3]*W)
				top    = int(detection[4]*H)
				right  = int(detection[5]*W)
				bottom = int(detection[6]*H)
				
				#draw a red rectangle around detected objects
				cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), thickness=2)
				

				tracker = cv2.legacy.TrackerKCF_create()
				bbox = [left, top, right-left, bottom-top]
				tracker.init(rgb,bbox)
				# add the tracker to our list of trackers so we can
				# utilize it during skip frames
				trackers.append(tracker)				
	else:
		dnnWork = 0
		i = 0
		# loop over the trackers
		for tracker in trackers:	
			# update the tracker and grab the updated position
			success, box = tracker.update(rgb)

			if success:
           
				left, top, width, height = [int(c) for c in box]
				right = int(left + width)
				bottom = int(top + height)
				print(left, top, right,bottom)
			#draw a red rectangle around detected objects
			cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), thickness=2)
			i +=1

	# show the output frame
	cv2.imshow("Frame", frame)

	if args["output"] is not None:
		# writing the video frame 
		result.write(frame)

	key = cv2.waitKey(1) & 0xFF
	# if the `q` key was pressed, break from the loop
	if key == ord("q"):

		break

	# increment the total number of frames processed up to now
	noFrames += 1
	# update the FPS counter
	fps.update()

# # stop the timer and display FPS information
fps.stop()
print("[INFO] : Elapsed time: {:.2f}".format(fps.elapsed()))
print("[INFO] : Approx. FPS:  {:.2f}".format(fps.fps()))

#=======================================================================
# release the video file pointer or video input stream
cap.release()


#=======================================================================
# close any open windows if exist
if args["output"] is None:
	print("[INFO] : Destroying the main graphical window.")
	cv2.destroyAllWindows()

#=======================================================================
# terminate all loop
mainHR_v.value = 0
threadGPS.join()
	
#=======================================================================
net.setPreferableTarget  (cv2.dnn.DNN_TARGET_CPU)
net.setPreferableBackend (cv2.dnn.DNN_BACKEND_OPENCV)	

print(" ")
print("[INFO] : The human recognition program finished!!!!")
