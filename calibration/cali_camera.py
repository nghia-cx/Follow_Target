import cv2, os
import numpy as np
import sys
current_path = os.getcwd()
sys.path.insert(0, current_path)
from config import *
# variables
# distance from camera to object(face) measured
KNOWN_DISTANCE = 100  # centimeter
# width of face in the real world or Object Plane
KNOWN_WIDTH = 170  # centimeter
# Colors
GREEN = (0, 255, 0)
RED = (0, 0, 255)
WHITE = (255, 255, 255)
fonts = cv2.FONT_HERSHEY_COMPLEX
cap = cv2.VideoCapture(0)

print("[INFO] : Loading dnn MobileNet model...")
net = cv2.dnn.readNetFromCaffe(configPath, weightsPath)
# focal length finder function
def focal_length(measured_distance, real_width, width_in_rf_image):

    focal_length_value = (width_in_rf_image * measured_distance) / real_width
    return focal_length_value


# distance estimation function
def distance_finder(focal_length, real_face_width, face_width_in_frame):

    distance = (real_face_width * focal_length) / face_width_in_frame
    return distance


# Calibratin for camera
while True:
    frame = cap.read()[1]
    frame = cv2.resize(frame,(640,480))

    (h,w)=frame.shape[:2]
    blob=cv2.dnn.blobFromImage(cv2.resize(frame,(300,300)),0.007843,(300,300),127.5)
    net.setInput(blob)
    detections=net.forward()
    focals = []
    for i in np.arange(0,detections.shape[2]):
        confidence=detections[0,0,i,2]
        if confidence>0.35:
            idx=int(detections[0,0,i,1])
            if idx != 15:
                continue
            box=detections[0,0,i,3:7]*np.array([w,h,w,h])
            (startX, startY, endX, endY) = box.astype("int")
            cv2.rectangle(frame,(startX,startY),(endX,endY),(0,255,0),2)
            img_height = endY - startY
            focal_length_found = focals.append(focal_length(KNOWN_DISTANCE, KNOWN_WIDTH, img_height))
    
    cv2.imshow("init", frame) 
    if cv2.waitKey(1) == ord("q"):
        print("focal_length: ",np.mean(focals))
        break      


# while True:

#     ret, frame = cap.read()
#     frame = cv2.resize(frame,(640,480))
#     (h,w)=frame.shape[:2]
#     blob=cv2.dnn.blobFromImage(cv2.resize(frame,(300,300)),0.007843,(300,300),127.5)
#     net.setInput(blob)
#     detections=net.forward()
#     for i in np.arange(0,detections.shape[2]):
#         confidence=detections[0,0,i,2]
#         if confidence>0.35:
#             idx=int(detections[0,0,i,1])
#             if idx != 15:
#                 continue
#             box=detections[0,0,i,3:7]*np.array([w,h,w,h])
#             (startX, startY, endX, endY) = box.astype("int")
#             img_height = endY - startY
#             focal_length_found = focal_length(KNOWN_DISTANCE, KNOWN_WIDTH, img_height)
#             # finding the distance by calling function Distance
#             if img_height != 0:
#                 Distance = distance_finder(focal_length_found, KNOWN_WIDTH, img_height)
#                 # Drwaing Text on the screen
#                 cv2.putText(frame, f"Distance = {round(Distance,2)} CM", (50, 50), fonts, 1, (255,0,0), 2)
#     cv2.imshow("frame", frame)
#     index += 1
#     if cv2.waitKey(1) == ord("q"):
#         break
# cap.release()
# cv2.destroyAllWindows()