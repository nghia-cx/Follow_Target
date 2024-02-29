from __future__ import with_statement
from __future__ import division
from __future__ import absolute_import
from os import sys, path
from io import open
sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
from Kalman_Filter_Custom_2D import KalmanFilter
#-- Dependencies for video processing
import time, os
import math
import argparse
import cv2
import numpy as np
import matplotlib.pyplot as plt
current_path = os.getcwd()
import sys
sys.path.insert(0, current_path)
from config import *

from imutils.video import FPS
#-- Dependencies for commanding the drone
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil

######-----------------------Dronekit-----------------------######
#-- Parse argument to connect the drone via terminal window
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

# Function to rotation via Yaw
def rotate_yaw(yaw_angle):
    # Gửi lệnh SET_ATTITUDE_TARGET
    vehicle.mav.set_attitude_target_send(
        vehicle.target_system,
        vehicle.target_component,
        time.time(),
        [0, 0, 0],  # Quay Roll và Pitch về 0
        yaw_angle,   # Góc Yaw mong muốn (quay quanh trục z)
        0,           # Tốc độ quay không đổi (có thể thay đổi nếu bạn muốn xoay nhanh chậm hơn)
        0            # Góc quay không đổi
    )



                
if __name__ == "__main__":

    #-- Initialize KCF tracking algorithm
    # tracker = cv2.legacy.TrackerKCF_create()


    fps = FPS().start() # Frames Per Second

    net = cv2.dnn.readNetFromCaffe(configPath, weightsPath)

    COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

    # Setup vehicle
    vehicle = connect_veh()

    #Takeoff height in meters
    arm_and_takeoff(2)
    print("Take off complete")

    # Hover for 10 seconds
    time.sleep(2)

    # Kalman filter initialization
    kf = KalmanFilter(dt, INIT_POS_STD, INIT_VEL_STD, ACCEL_STD, GPS_POS_STD)

    
    try:

        cap = cv2.VideoCapture(0) # capture video from web camera
        #-- Start saving output video
        # fourcc = cv2.VideoWriter_fourcc(*'MP4V')
        # out = cv2.VideoWriter(u"output_pred.mp4", fourcc, 15.0,(640,480))
        while True:
            timer=cv2.getTickCount()
            ret, frame = cap.read()

            (h, w) = frame.shape[:2]
            
            blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)),
                0.007843, (300, 300), 127.5)


            net.setInput(blob)
            detections = net.forward()
            list_obj = []
            iou_list = []
            centers = []
            # loop over the detections
            for i in np.arange(0, detections.shape[2]):

                confidence = detections[0, 0, i, 2]
                if confidence > 0.6:

                    idx = int(detections[0, 0, i, 1])

                    box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                    (startX, startY, endX, endY) = box.astype("int")
                    list_obj.append([startX, startY, endX, endY])

                    # draw the prediction on the frame
                    if tag:
                        label = "{}: {:.2f}%".format(CLASSES[idx],
                            confidence * 100)
                        cv2.rectangle(frame, (startX, startY), (endX, endY),
                            COLORS[idx], 2)
                        y = startY - 15 if startY - 15 > 15 else startY + 15
                        cv2.putText(frame, label, (startX, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)

                
            # print("list_obj",list_obj)
            if list_obj == []:
                rotate_yaw()
            if roi is not None:
                cv2.putText(frame, 'TRACKING!!!', (400, 30), font, fontScale, (0, 0, 255), thickness, cv2.LINE_AA)
                roi = list(roi)
                roi = [roi[0], roi[1], roi[0] + roi[2], roi[1] + roi[3]]

                for a in list_obj:
                    iou_list.append(iou(roi, a))
                # print("list_iou", iou_list)
                index_of_max = iou_list.index(max(iou_list))
                roi = list_obj[index_of_max]
                cv2.rectangle(frame, (roi[0], roi[1]), (roi[2], roi[3]), (0, 255, 0), 2)
                wb = roi[2]-roi[0]
                hb = roi[3]-roi[1]
                centerX = int((roi[0]+wb/2))
                centerY = int((roi[1]+hb/2))
                centers.append((centerX,centerY))
                cv2.circle(frame,(centerX,centerY),10,(0,255,0),-1)
                # cv2.line(frame,(int(640/2),int(480/2)), (centerX,centerY), (255,255,255), 3)
                iou_list = []

                if len(centers) > 0:
            
                    center = centers[0]  # Extract the first center tuple

                    # Example: Draw circle at the center
                    if isinstance(center, tuple):
                        # print("Center = ", center)
                        cv2.circle(frame, center, radius=8, color=(0, 255, 0), thickness=4) # Green

                        x_pred, y_pred = kf.predict()
                        if isFirstFrame:  # First frame
                            x_pred = round(x_pred[0])
                            y_pred = round(y_pred[0])
                            # print("Predicted: ", (x_pred, y_pred))
                            isFirstFrame = False
                        else:
                            x_pred = round(x_pred[0])
                            y_pred = round(y_pred[1])
                            # print("Predicted: ", (x_pred, y_pred))

                        # cv2.circle(frame, (x_pred, y_pred), radius=8, color=(255, 0, 0), thickness=4) #  Blue

                        # Update
                        (x1, y1) = kf.update(center)
                        x_updt = round(x1[0])
                        y_updt =  round(x1[1])
                        print("Coordinate: ", (x_updt, y_updt))
                cv2.circle(frame, (x_updt, y_updt), radius=8, color= (0, 0, 255), thickness=4) # Red
                cv2.line(frame,(int(640/2),int(480/2)), (x_updt,y_updt), (255,255,255), 3)

                # Estimate distance
                distance = 150*(170/(roi[2]-roi[0]))
                # print(box)
                cv2.putText(frame,"IDISTANCE: {distance}cm".format(distance=int(distance)), (30,30),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,255),2)
                cv2.circle(frame,(320,240),10,(0,0,255),-1)
                ######-----------------------Controlling drone-----------------------######
                
                #-- Fly forward and backward
                if 60<=distance<=100:
                    send_local_ned_velocity(0,0,0) #keep in place
                    cv2.putText(frame,"KEEP DISTANCE", (30,50),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,255),2)
                    save = True
                else:
                    save = False
                    if distance > 100:
                        send_local_ned_velocity(0.5,0,0) # x is positive number so the drone flying forward 1m/s in 0.5 second duration
                        cv2.putText(frame,"FORWARD", (30,50),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,255),2)
                    elif distance <60:
                        send_local_ned_velocity(-0.5,0,0) # x is negative number so the drone flying backward 1m/s in 0.5 second duration
                        cv2.putText(frame,"BACKWORD", (30,50),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,255),2)
                if save:
                    if (300 <= int((startX+wb/2)) <=340 and 200 <= int((startY+hb/2)) <= 260): 
                        send_local_ned_velocity(0,0,0) #keep in place
                        cv2.putText(frame,"KEEP STABLE", (30,70),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,255),2)
                    else:
                        #-- Fly right and left
                        if int((startX+wb/2)) > 340: # person is in the right so it needs to fly right
                            send_local_ned_velocity(0,0.5,0) # y is positive number so the drone flying right 1m/s in 0.5 second duration
                            cv2.putText(frame,"MOVE RIGHT", (30,70),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,255),2)
                        
                        elif int((startX+wb/2)) < 300: # person is in the left so it needs to fly left
                            send_local_ned_velocity(0,-0.5,0) # y is negative number so the drone flying left 1m/s in 0.5 second duration
                            cv2.putText(frame,"MOVE LEFT", (30,70),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,255),2)
                            
                        # #-- Fly high and low
                        # if int((startY+hb/2)) > 260: # person is in the high so it needs to fly high
                        #     send_local_ned_velocity(0,0,0.5) # z is positive number so the drone flying high 1m/s in 0.5 second duration
                        #     cv2.putText(frame,"LOW", (30,90),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,255),2)
                        #     
                        # elif int((startY+hb/2)) < 220: # person is in the low so it needs to fly low
                        #     send_local_ned_velocity(0,0,-0.5) # z is negative number so the drone flying low 1m/s in 0.5 second duration
                        #     cv2.putText(frame,"HIGH", (30,90),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,255),2)
                        # 
                # Draw trajectory map
                drone_trajectory.append((int((roi[0] + roi[2]) / 2), int((roi[1] + roi[3]) / 2)))
                human_trajectory.append((int((roi[0] + wb / 2)), int((roi[1] + hb / 2))))   
            else:
                cv2.putText(frame, 'LOSE TRACK!', (400, 30), font, fontScale, (0, 255, 0), thickness, cv2.LINE_AA) 
            totalFrames+=1

            #fps
            fps.update()

            fps1 = cv2.getTickFrequency()/(cv2.getTickCount()-timer)
            # cv2.putText(img,unicode(int(fps1)),(75,50),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
            # out.write(img)

            cv2.imshow('Tracking',frame)
            key = cv2.waitKey(1) & 0xFF

            # if the `q` key was pressed, break from the loop
            if key == ord("q"):
                break
            if key == ord("a"):
                print("a pressed >> Set the vehicle to LAND")
                vehicle.mode = VehicleMode("LAND")
            if key == ord("r"):
                print("r pressed >> Set the vehicle to RTL")
                vehicle.mode = VehicleMode("RTL")
            if key == ord("s"):
                tag = False
                roi = cv2.selectROI("Tracking", frame)

                print(roi)
                # tracker.init(frame, roi)

        fps.stop()
        print ("[INFO] elapsed time: {:.2f}".format(fps.elapsed()))
        print ("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
        plt.figure(figsize=(8, 6))
        drone_x, drone_y = zip(*drone_trajectory)
        human_x, human_y = zip(*human_trajectory)
        plt.plot(drone_x, drone_y, label='Drone')
        plt.plot(human_x, human_y, label='Human')
        plt.title('Drone and Human Trajectories')
        plt.xlabel('X-axis')
        plt.ylabel('Y-axis')
        plt.legend()
        plt.grid(True)
        plt.show()
    finally:
        # shut down cleanly
        # out.release()
        cap.release()
        cv2.destroyAllWindows()