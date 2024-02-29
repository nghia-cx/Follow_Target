import numpy as np
import cv2
import os
import curses
import time
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
import argparse
import sys
current_path = os.getcwd()
sys.path.insert(0, current_path)
from config import *

net = cv2.dnn.readNetFromCaffe(configPath, weightsPath)
#facedet = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

tracker = cv2.legacy.TrackerKCF_create()


# ==========================================================================
def setServoAngle(servo, angle):
	assert angle >=0 and angle <= 180
	pwm = GPIO.PWM(servo, 50)
	pwm.start(8)
	dutyCycle = angle / 18. + 3.
	pwm.ChangeDutyCycle(dutyCycle)
	time.sleep(0.3)
	pwm.stop()
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
        iou = 0

    return iou

try:
    setServoAngle(pan, current_PAN)
    setServoAngle(tilt, current_TILT)
    cap=cv2.VideoCapture(0)
    while True:

        ret,frame=cap.read()
        frame=cv2.resize(frame,(640,480))
        
        (h,w)=frame.shape[:2]
        blob=cv2.dnn.blobFromImage(cv2.resize(frame,(300,300)),0.007843,(300,300),127.5)
        net.setInput(blob)
        detections=net.forward()
        list_obj = []
        iou_list = []
        for i in np.arange(0,detections.shape[2]):
            confidence=detections[0,0,i,2]
            if confidence>0.5:
                idx=int(detections[0,0,i,1])
                box=detections[0,0,i,3:7]*np.array([w,h,w,h])
                (startX,startY,endX,endY)=box.astype('int')
                list_obj.append([startX, startY, endX, endY])
                if tag:
                    label = "{}: {:.2f}%".format(CLASSES[idx], confidence * 100)
                    cv2.rectangle(frame,(startX,startY),(endX,endY),COLORS[idx],2)
                    y=startY-15 if startY-15>15 else startY+15
                    cv2.putText(frame,label,(startX,y),cv2.FONT_HERSHEY_SIMPLEX,0.5,COLORS[idx],2)
                
        #print(frame.shape[0])
        #print(frame.shape[1])
        #frame=cv2.flip(frame,1)
        cv2.circle(frame, (frame.shape[1]//2,frame.shape[0]//2), 5, (0, 0, 255), thickness=-1, lineType=1, shift=0)
        if list_obj == []:
            continue
        if roi is not None:
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
            cv2.putText(frame, 'TRACKING!!!', (20, 30), font, fontScale, (0, 0, 255), thickness, cv2.LINE_AA)


            print("x, y previous: ",previous_x,previous_y)
            delta_x = frame.shape[1]//2 - (roi[0] + wb//2)
            delta_y = frame.shape[0]//2 - (roi[1] + hb//2)    
            
            delta_x_dot = delta_x - previous_delta_x
            delta_y_dot = delta_y - previous_delta_y

            if abs(delta_x) < error_acceptance:
                delta_x     = 0
                delta_x_dot = 0
                
            if abs(delta_y) < error_acceptance:
                delta_y     = 0
                delta_y_dot = 0

            print('x, y new: ', roi[0], roi[1])
            
            previous_x = roi[0] + wb//2
            previous_y = roi[1] + hb//2
            
            previous_h = h
            previous_w = w
            
            previous_delta_x = delta_x
            previous_delta_y = delta_y
            
            print('pan, tilt current: ', current_PAN, current_TILT)

            # pseu-do PID
            delta_TILT = k_TILT * delta_y + kd_TILT * delta_y_dot
            # rate-limiter
            delta_TILT = min(abs(delta_TILT), max_rate_TILT)*np.sign(delta_TILT)
            # noise exclude
            if abs(delta_TILT) < step_TILT:
                delta_TILT = 0
            # here we go
            current_TILT = int(current_TILT + delta_TILT)

            
            if current_TILT > max_TILT:
                current_TILT = max_TILT                
            if current_TILT < min_TILT:                
                current_TILT = min_TILT
                
            print('delta_tilt: ', delta_TILT)
            # pseu-do PID
            delta_PAN = k_PAN * delta_x + kd_PAN * delta_x_dot
            # rate-limiter
            delta_PAN = min(abs(delta_PAN), max_rate_PAN)*np.sign(delta_PAN)
            # noise exclude
            if abs(delta_PAN) < step_PAN:
                delta_PAN = 0            
            # here we go
            
            current_PAN = int(current_PAN + delta_PAN)
                
            if current_PAN > max_PAN:
                current_PAN = max_PAN                
            if current_PAN < min_PAN:                
                current_PAN = min_PAN           
            
            print('delta_PAN: ', delta_PAN)
            
            print('delta_x, delta_y: ', delta_x, delta_y)
            
            print('delta_x_dot, delta_y_dot: ', delta_x_dot, delta_y_dot)
            
            print('pan, tilt new: ', current_PAN, current_TILT)
            
            print("----------------------------------------")
            
            #pwm.setRotationAngle(1, current_PAN)
            #pwm.setRotationAngle(0, current_TILT)
            setServoAngle(pan, current_PAN)
            setServoAngle(tilt, current_TILT)
            time.sleep(1)

        else:
            cv2.putText(frame, 'LOSE TRACK!', (20, 30), font, fontScale, (0, 255, 0), thickness, cv2.LINE_AA)
        # wait for keypress
        # ===========================================================
        cv2.imshow("Tracking",frame)
        char = cv2.waitKey(20)
        #print('key pressed', char)
        
        if char == ord('q'):
            break
        if char == ord('s'):
            tag = False
            roi = cv2.selectROI('Tracking',frame)
            print(roi)
            tracker.init(frame,roi)
        elif char == 83:
            current_PAN = max(min_PAN, current_PAN - step_PAN)
            #pwm.setRotationAngle(1, current_PAN) #PAN
            setServoAngle(pan, current_PAN)
            #time.sleep(0.001)
            #cv2.putText(frame, 'right ', (20, 20), font, fontScale, (0, 255, 0), thickness, cv2.LINE_AA) 
            
        elif char == 81:
            current_PAN = min(max_PAN, current_PAN + step_PAN)
            #pwm.setRotationAngle(1, current_PAN) #PAN
            setServoAngle(pan, current_PAN)
            #time.sleep(0.001)
            #cv2.putText(frame, 'left ', (20, 20), font, fontScale, (0, 255, 0), thickness, cv2.LINE_AA)
            
        elif char == 82:
            current_TILT = max(min_TILT, current_TILT - step_TILT)
            #pwm.setRotationAngle(0, current_TILT) #TILT
            setServoAngle(tilt, current_TILT)
            #time.sleep(0.001)
            #cv2.putText(frame, 'up ', (20, 20), font, fontScale, (0, 255, 0), thickness, cv2.LINE_AA)
            
        elif char == 84:
            current_TILT = min(max_TILT, current_TILT + step_TILT)
            #pwm.setRotationAngle(0, current_TILT) #TILT
            setServoAngle(tilt, current_TILT)
            #time.sleep(0.001)            
            #cv2.putText(frame, 'down ', (20, 20), font, fontScale, (0, 255, 0), thickness, cv2.LINE_AA)
            
#             
finally:
    # shut down cleanly
    #pwm.exit_PCA9685()
    GPIO.cleanup()
    cap.release()
    cv2.destroyAllWindows()
