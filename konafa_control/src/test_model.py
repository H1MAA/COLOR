#!/usr/bin/env python

from ultralytics import YOLO
import cv2

cap = cv2.VideoCapture(0)
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))


model_path = 'models/yolov8m.pt'

model = YOLO(model_path)

frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

while cap.isOpened():
    ret,frame = cap.read()
    result = model.track(source=frame, show=True, iou=0.5 , conf = 0.01  , save = False ,verbose = False,show_labels=True)  
    xMid = frame_width//2
    yMid = frame_height//2
    cv2.circle(frame, (xMid+110,yMid), radius=5, color=(0,0,255), thickness=-1)
    cv2.imshow("konafa",frame)

    if cv2.waitKey(10) & 0xFF == ord('q'):
            break
    
cap.release()
cv2.destroyAllWindows()