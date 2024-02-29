#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import time, argparse
import os
import importlib.util

#- Importing Tkinter: sudo apt-get install python-tk
import tkinter as tk
class ImageSubscriber:
    def __init__(self):
        rospy.init_node('image_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/roscam/cam/image_raw', Image, self.image_callback)

    def image_callback(self, data):

        self.cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        frame = self.cv_image
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb, (width, height))
        input_data = np.expand_dims(frame_resized, axis=0)
        
        # Normalize pixel values if using a floating model (i.e. if model is non-quantized)
        if floating_model:
            input_data = (np.float32(input_data) - input_mean) / input_std
      
        # Perform the actual detection by running the model with the image as input
        interpreter.set_tensor(input_details[0]['index'],input_data)
        interpreter.invoke()

        # Retrieve detection results
        boxes = interpreter.get_tensor(output_details[boxes_idx]['index'])[0] # Bounding box coordinates of detected objects
        classes = interpreter.get_tensor(output_details[classes_idx]['index'])[0] # Class index of detected objects
        scores = interpreter.get_tensor(output_details[scores_idx]['index'])[0] # Confidence of detected objects

        # Loop over all detections and draw detection box if confidence is above minimum threshold
        for i in range(len(scores)):
            if ((scores[i] > min_conf_threshold) and (scores[i] <= 1.0)):

                # Get bounding box coordinates and draw box
                # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
                ymin = int(max(1,(boxes[i][0] * imH)))
                xmin = int(max(1,(boxes[i][1] * imW)))
                ymax = int(min(imH,(boxes[i][2] * imH)))
                xmax = int(min(imW,(boxes[i][3] * imW)))
                
                cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), (10, 255, 0), 4)

                # Draw label
                object_name = labels[int(classes[i])+1] # Look up object name from "labels" array using class index
                label = '%s: %d%%' % (object_name, int(scores[i]*100)) # Example: 'person: 72%'
                labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2) # Get font size
                label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
                cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
                cv2.putText(frame, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2) # Draw label text
                # cv2.circle(frame,(320,240),10,(0,0,255),-1)
                # cv2.circle(frame,(xmin + int((xmax-xmin)/2),ymin+int((ymax-ymin)/2)),10,(0,0,255),-1)
                # cv2.line(frame,(int(640/2),int(480/2)), (xmin + int((xmax-xmin)/2,ymin+int((ymax-ymin)/2)), (255,255,255), 3))

 

                    ######-----------------------Controlling drone-----------------------######
        #     if list_obj == []:
        #         continue
        #     if roi is not None:
                
        #         roi = list(roi)
        #         roi = [roi[0], roi[1], roi[0] + roi[2], roi[1] + roi[3]]

        #         for a in list_obj:
        #             iou_list.append(iou(roi, a))
        #         print("list_iou", iou_list)
        #         index_of_max = iou_list.index(max(iou_list))
        #         roi = list_obj[index_of_max]
        #         cv2.rectangle(frame, (roi[0], roi[1]), (roi[2], roi[3]), (0, 255, 0), 2)
        #         wb = roi[2]-roi[0]
        #         hb = roi[3]-roi[1]
        #         cv2.circle(frame,(int((roi[0]+wb/2)),int((roi[1]+hb/2))),10,(0,255,0),-1)
        #         cv2.line(frame,(int(640/2),int(480/2)), (int((roi[0]+wb/2)),int((roi[1]+hb/2))), (255,255,255), 3)
        #         iou_list = []
                # Estimate distance
                distance = 150*(150/(ymax-ymin))
                # print(box)
                cv2.putText(frame,"Estimated distance {distance}cm".format(distance=int(distance)), (30,30),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,255),2)
                
        
                #-- Fly forward and backward
                if 100<=distance<=150:
                    send_local_ned_velocity(0,0,0) #keep in place
                    cv2.putText(frame,"KEEP DISTANCE", (30,50),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,255),2)
                    save = True
                else:
                    save = False
                    if distance > 150:
                        send_local_ned_velocity(0.5,0,0) # x is positive number so the drone flying forward 1m/s in 0.5 second duration
                        cv2.putText(frame,"FORWARD", (30,50),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,255),2)
                    elif distance <100:
                        send_local_ned_velocity(-0.5,0,0) # x is negative number so the drone flying backward 1m/s in 0.5 second duration
                        cv2.putText(frame,"BACKWORD", (30,50),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,255),2)
                if save:
                    #- Fly keep stable
                    if (300 <= int((xmin+(xmax-xmin)/2)) <=360 and 200 <= int((ymin+(ymax-ymin)/2)) <= 240): 
                        send_local_ned_velocity(0,0,0) #keep in place
                        cv2.putText(frame,"KEEP STABLE", (30,70),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,255),2)
                    else:
                        #-- Fly right and left
                        if int((xmin+(xmax-xmin)/2)) > 360: # person is in the right so it needs to fly right
                            send_local_ned_velocity(0,0.5,0) # y is positive number so the drone flying right 1m/s in 0.5 second duration
                            cv2.putText(frame,"RIGHT", (30,70),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,255),2)
                            continue
                        elif int((xmin+(xmax-xmin)/2)) < 240: # person is in the left so it needs to fly left
                            send_local_ned_velocity(0,-0.5,0) # y is negative number so the drone flying left 1m/s in 0.5 second duration
                            cv2.putText(frame,"LEFT", (30,70),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,255),2)
                            continue
                        #-- Fly high and low
                        # elif int((ymin+(ymax-ymin)/2)) > 260: # person is in the high so it needs to fly high
                        #     send_local_ned_velocity(0,0,0.5) # z is positive number so the drone flying high 1m/s in 0.5 second duration
                        #     cv2.putText(frame,"LOW", (30,90),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,255),2)
                        #     continue
                        # elif int((ymin+(ymax-ymin)/2)) < 220: # person is in the low so it needs to fly low
                        #     send_local_ned_velocity(0,0,-0.5) # z is negative number so the drone flying low 1m/s in 0.5 second duration
                        #     cv2.putText(frame,"HIGH", (30,90),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,255),2)
                        #     continue 
        cv2.imshow('Iris Drone Camera', frame)
        # out.write(frame)                  

        key = cv2.waitKey(1)
        # if key == ord('s'):
        #     # tag = False
        #     roi = cv2.selectROI("Iris Drone Camera", frame)

        #     print(roi)


    def get_frame(self):
        return self.cv_image
    
def iou(box1, box2):

    if box1[0] < box2[2] and box1[2] > box2[0] and box1[1] < box2[3] and box1[3] > box2[1]:
        # cv2.putText(image, "Giao",(10,20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),2)

        xi1 = max(box1[0], box2[0])
        yi1 = max(box1[1], box2[1])
        xi2 = min(box1[2], box2[2])
        yi2 = min(box1[3], box2[3])
        inter_area = abs(yi2 - yi1) * (xi2 - xi1)

        box1_area = (box1[2] - box1[0]) * (box1[3] - box1[1])
        box2_area = (box2[2] - box2[0]) * (box2[3] - box2[1])
        union_area = box1_area + box2_area - inter_area

        # compute the IoU
        iou = inter_area / union_area
    else:
        # cv2.putText(image, "Ko giao", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        iou = 0

    return iou

def connect_veh():
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect', default='127.0.0.1:14550')
    args = parser.parse_args()

    #-- Connect to the drone
    print ('Connecting...')
    vehicle = connect(args.connect, baud=921600) 
    return vehicle 

# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(aTargetAltitude):

    print ("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print (" Waiting for vehicle to initialise...")
        time.sleep(1)


    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print ("........")
        time.sleep(1)

    print ("ARMED")
    print("Taking Off!")

    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude


#-- Function to controll velocity of the drone
def send_local_ned_velocity(x, y, z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        __builtins__.int("0000111111000111", 2), 
        0, 0, 0, 
        x, y, z, # x, y, z velocity in m/s
        0, 0, 0,
        0, 0)    

    # send command to the drone
    vehicle.send_mavlink(msg)
    vehicle.flush()


if __name__ == '__main__':
    # Define and parse input arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--modeldir', help=r"Name of the .tflite file, if different than detect.tflite",
                        required=True)
    parser.add_argument('--graph', help='Name of the .tflite file, if different than detect.tflite',
                        default='detHuman1.tflite')
    parser.add_argument('--labels', help='Name of the labelmap file, if different than labelmap.txt',
                        default=r"CodeDATN\Document\Code\Code_Follow_Target\TF_lite\label_map.txt")
    parser.add_argument('--threshold', help='Minimum confidence threshold for displaying detected objects',
                        default=0.5)
    # parser.add_argument('--video', help='Name of the video file',
    #                     default=r"/home/nghiacx310/Documents/Python/output2.mp4")
    parser.add_argument('--edgetpu', help='Use Coral Edge TPU Accelerator to speed up detection',
                        action='store_true')

    parser.add_argument('--connect', default='127.0.0.1:14550')
    args = parser.parse_args()


    MODEL_NAME = args.modeldir
    GRAPH_NAME = args.graph
    LABELMAP_NAME = args.labels
    # VIDEO_NAME = args.video
    min_conf_threshold = float(args.threshold)
    use_TPU = args.edgetpu

    # Import TensorFlow libraries
    # If tflite_runtime is installed, import interpreter from tflite_runtime, else import from regular tensorflow
    # If using Coral Edge TPU, import the load_delegate library
    pkg = importlib.util.find_spec('tflite_runtime')
    if pkg:
        from tflite_runtime.interpreter import Interpreter
        if use_TPU:
            from tflite_runtime.interpreter import load_delegate
    else:
        from tensorflow.lite.python.interpreter import Interpreter
        if use_TPU:
            from tensorflow.lite.python.interpreter import load_delegate

    # If using Edge TPU, assign filename for Edge TPU model
    if use_TPU:
        # If user has specified the name of the .tflite file, use that name, otherwise use default 'edgetpu.tflite'
        if (GRAPH_NAME == 'detect.tflite'):
            GRAPH_NAME = 'edgetpu.tflite'   

    # Get path to current working directory
    CWD_PATH = os.getcwd()

    # Path to video file
    # VIDEO_PATH = os.path.join(CWD_PATH,VIDEO_NAME)

    # Path to .tflite file, which contains the model that is used for object detection
    PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,GRAPH_NAME)

    # Path to label map file
    PATH_TO_LABELS = os.path.join(CWD_PATH,MODEL_NAME,LABELMAP_NAME)

    # Load the label map
    with open(PATH_TO_LABELS, 'r') as f:
        labels = [line.strip() for line in f.readlines()]

    # Have to do a weird fix for label map if using the COCO "starter model" from
    # https://www.tensorflow.org/lite/models/object_detection/overview
    # First label is '???', which has to be removed.
    if labels[0] == '???':
        del(labels[0])

    # Load the Tensorflow Lite model.
    # If using Edge TPU, use special load_delegate argument
    if use_TPU:
        interpreter = Interpreter(model_path=PATH_TO_CKPT,
                                experimental_delegates=[load_delegate('libedgetpu.so.1.0')])
        print(PATH_TO_CKPT)
    else:
        interpreter = Interpreter(model_path=PATH_TO_CKPT)

    interpreter.allocate_tensors()

    # Get model details
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    height = input_details[0]['shape'][1]
    width = input_details[0]['shape'][2]
    
    floating_model = (input_details[0]['dtype'] == np.float32)

    input_mean = 127.5
    input_std = 127.5

    # Check output layer name to determine if this model was created with TF2 or TF1,
    # because outputs are ordered differently for TF2 and TF1 models
    outname = output_details[0]['name']

    if ('StatefulPartitionedCall' in outname): # This is a TF2 model
        boxes_idx, classes_idx, scores_idx = 1, 3, 0
    else: # This is a TF1 model
        boxes_idx, classes_idx, scores_idx = 0, 1, 2

    # # Open video file
    # video = cv2.VideoCapture(VIDEO_PATH)
    # imW = video.get(cv2.CAP_PROP_FRAME_WIDTH)
    # imH = video.get(cv2.CAP_PROP_FRAME_HEIGHT)
    imW = 640
    imH = 480

    #-- Connect to the drone
    print ('Connecting to drone...')
    vehicle = connect(args.connect, baud=921600) 
    arm_and_takeoff(2)
    print("Take off complete")

    time.sleep(2)
    #-- Start saving output video
    # fourcc = cv2.VideoWriter_fourcc(*'XVID')
    # out = cv2.VideoWriter('output3.avi',fourcc,30, (640,480))
    roi = None
    save = False
    image_subscriber = ImageSubscriber()
    rospy.spin()

    cv2.destroyAllWindows()
