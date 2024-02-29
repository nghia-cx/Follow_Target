import cv2, time
import numpy as np
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

GPIO.setup(35, GPIO.OUT)
GPIO.setup(36, GPIO.OUT)
GPIO.setup(37, GPIO.OUT)
GPIO.setup(38, GPIO.OUT)

m1 = GPIO.PWM(35, 50)
m2 = GPIO.PWM(36, 50)
m3 = GPIO.PWM(37, 50)
m4 = GPIO.PWM(38, 50)

m1.start(0)
m2.start(0)
m3.start(0)
m4.start(0)

print ("starting 0")
time.sleep(3)

m1.ChangeDutyCycle(3)
m2.ChangeDutyCycle(3)
m3.ChangeDutyCycle(3)
m4.ChangeDutyCycle(3)

print("start")
time.sleep(5)

#==============================================#

tracker = cv2.legacy.TrackerKCF_create()

# ========= Variables =========#
ROI_dif = 100
counter = 0
Move_Dir = "null"
FrameCentre = 640
PreArea = 0
# =============================#
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
# ===== Getting Video =====#
#cap = cv2.VideoCapture("/home/raspi/Documents/Code/Track.avi")
cap = cv2.VideoCapture(0)

'''fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter('TrackAI.avi',fourcc, 3.2, (640,480), True)'''
# =========================#
# =========== Obj_Detection_Config ===========#
# Setup model
configPath = 'MobileNetSSD_deploy.prototxt.txt'
weightsPath = 'MobileNetSSD_deploy.caffemodel'
net = cv2.dnn.readNetFromCaffe(configPath, weightsPath)
CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
"bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
"dog", "horse", "motorbike", "person", "pottedplant", "sheep",
"sofa", "train", "tvmonitor"]
COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))
# ============================================#

#RollIN = 50  # note:
#PitchIN = 50 # note:
roi = None
tag = True
save = False
def iou(box1, box2):

    if box1[0] < box2[2] and box1[2] > box2[0] and box1[1] < box2[3] and box1[3] > box2[1]:

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

while True:

    ret, frame = cap.read()
    frame = cv2.resize(frame,(640,480))
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
        continue
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

        # Estimate distance
        distance = 150*(170/(roi[2]-roi[0]))
        cv2.putText(frame,"Estimated distance {distance}cm".format(distance=int(distance)), (30,30),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,255),2)
        cv2.circle(frame,(320,240),10,(0,0,255),-1)
        cv2.line(frame,(int(640/2),int(480/2)), (centerX,centerY), (255,255,255), 3)
        #-- Fly forward and backward
        if 100<=distance<=150:
            cv2.putText(frame,"KEEP DISTANCE", (30,50),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,255),2)
            m1.ChangeDutyCycle(5)
            m2.ChangeDutyCycle(5)
            m3.ChangeDutyCycle(5)
            m4.ChangeDutyCycle(5)
            save = True
        else:
            save = False
            if distance > 150:
                cv2.putText(frame,"FORWARD", (30,50),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,255),2)
                m1.ChangeDutyCycle(5)
                m2.ChangeDutyCycle(7)
                m3.ChangeDutyCycle(5)
                m4.ChangeDutyCycle(7)
            elif distance <100:
                cv2.putText(frame,"BACKWORD", (30,50),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,255),2)
                m1.ChangeDutyCycle(7)
                m2.ChangeDutyCycle(5)
                m3.ChangeDutyCycle(7)
                m4.ChangeDutyCycle(5)
        '''if save:
            if (300 <= int(centerX) <=340 and 200 <= int(centerY) <= 260): 
                cv2.putText(frame,"KEEP STABLE", (30,70),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,255),2)
                m1.ChangeDutyCycle(5)
                m2.ChangeDutyCycle(5)
                m3.ChangeDutyCycle(5)
                m4.ChangeDutyCycle(5)
            else:
                #-- Fly right and left
                if int(centerX) > 340: # person is in the right so it needs to fly right
                    cv2.putText(frame,"RIGHT", (30,70),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,255),2)
                    m1.ChangeDutyCycle(5)
                    m2.ChangeDutyCycle(7)
                    m3.ChangeDutyCycle(7)
                    m4.ChangeDutyCycle(5)
                elif int(centerX) < 300: # person is in the left so it needs to fly left
                    cv2.putText(frame,"LEFT", (30,70),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,255),2)
                    m1.ChangeDutyCycle(7)
                    m2.ChangeDutyCycle(5)
                    m3.ChangeDutyCycle(5)
                    m4.ChangeDutyCycle(7)
                #-- Fly high and low
                # if int(y_updt) > 260: # person is in the high so it needs to fly high
                #     cv2.putText(frame,"LOW", (30,90),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,255),2)
                    
                # elif int(y_updt) < 220: # person is in the low so it needs to fly low
                #     cv2.putText(frame,"HIGH", (30,90),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,0,255),2)
    '''
    cv2.imshow("Tracking",frame)
    # out.write(img)
    key = cv2.waitKey(1)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    if key == ord("q"):
            break

    if key == ord("s"):
        tag = False
        roi = cv2.selectROI("Tracking", frame)

        print(roi)
        # tracker.init(frame, roi)
    if key == ord("c"):
        roi = None
        tag = True

cap.release()
# out.release() # save whole video on raspberry pi storage
cv2.destroyAllWindows()