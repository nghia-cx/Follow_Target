import cv2
from multiprocessing import Value
import numpy as np
#-- Parameters for detection model
whT = 320 #width and height for target
confidenceThreshold = 0.8
nmsThreshold =0.3
index = 0
confidence = 1.5
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
"bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
"dog", "horse", "motorbike", "person", "pottedplant", "sheep",
"sofa", "train", "tvmonitor"]
COLORS=np.random.uniform(0,255,size=(len(CLASSES),3))
configPath = 'mobilenet/MobileNetSSD_deploy.prototxt.txt'
weightsPath = 'mobilenet/MobileNetSSD_deploy.caffemodel'

# Kalman filter parameters
dt = 1/30  # Sampling time = FPS
INIT_POS_STD = 10  # Initial position standard deviation
INIT_VEL_STD = 10  # Initial velocity standard deviation
ACCEL_STD = 40  # Acceleration standard deviation
GPS_POS_STD = 1  # Measurement position standard deviation
isFirstFrame = True

# Parameter for control gimbal
max_PAN     = 180
max_TILT     = 145
min_PAN      = 0
min_TILT     = 0

max_rate_TILT = 3
max_rate_PAN  = 3
    
step_PAN     = 2
step_TILT    = 1
current_PAN  = 90
current_TILT = 90
#pwm.setRotationAngle(1, current_PAN) #PAN    
#pwm.setRotationAngle(0, current_TILT) #TILT
pan = 27
tilt = 17


# pseudo-PID control
k_PAN = 0.015
k_TILT = -0.015

kd_PAN = 0.095
kd_TILT = -0.095

error_acceptance = 15

previous_x = 0
previous_y = 0

previous_h = 0
previous_w = 0                  

delta_x = 0
delta_y = 0

previous_delta_x = 0
previous_delta_y = 0

delta_x_dot = 0
delta_y_dot = 0
angle = 0

# font 
font = cv2.FONT_HERSHEY_SIMPLEX  
# org 
org = (50, 50)   
# fontScale 
fontScale = 1   
# Blue color in BGR 
color = (255, 0, 0)   
# Line thickness of 2 px 
thickness = 2

roi = None
tag = True
save = False

totalFrames = 0 # to count frames
skip = 30 # skip frames which means after every 30 frames, detecting algorithm will detect a person

drone_trajectory = []
human_trajectory = []

#0 to 100, higher is better quality, 95 is cv2 default
jpeg_quality = 65 
skip_frames  = 15

# the number of frames after a custom MAVlink message will be sent 
customMess_frames = 40

# INIT FOR TRACKING ALGORITHM
#=====================================================
# initialize the frame dimensions
W = None
H = None

# initialize the number of frames processed up to now
noFrames    = 0
confidence  = 0
myLatitude  = 0
myLongitude = 0

writerProcess = None
streamProcess = None

threadGPS     = None

writeVideo_v  = None
streamVideo_v = None
threadGPS_v   = None	
mainHR_v      = None	

getgpsQueue   = None	# the GPS data frame queue
frameWQueue   = None	# the frame queue for avi file writing
frameSQueue   = None	# the frame queue for the video streaming

mainHR_v      = Value('i', 1)