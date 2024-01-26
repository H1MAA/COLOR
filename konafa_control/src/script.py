#!/usr/bin/env python
from Centralization import Centralization_Processing
from ultralytics import YOLO
import cv2

counter_display = 0

def centralize_omega():
    global counter_display
    cap = cv2.VideoCapture(0)
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    centralization.frame_width = frame_width
    centralization.frame_height = frame_height

    while cap.isOpened():
        ret,frame = cap.read()
        result = model.track(source=frame, show=True, iou=0.5 , conf = 0.5  , save = False , verbose = False,show_labels=True)          
        Robot_center = (frame_width//2,frame_height//2)
        centralization.Robot_center = Robot_center
        should_update = centralization.Update_errors(result)

        frame2 = cv2.circle(frame, (Robot_center[0],Robot_center[1]), radius=5, color=(255,0,0), thickness=-1)
        print(f"lat error = {centralization.Lateral_error}")
        if centralization.Lateral_error:
            cv2.circle(frame2, (int(centralization.X_center),Robot_center[1]), radius=5, color=(0,0,0), thickness=-1)
            cv2.arrowedLine(frame2,(Robot_center[0],Robot_center[1]),(int(centralization.X_center),Robot_center[1]),(0,255,0),2)
            cv2.line(frame2,(int(centralization.X_center),Robot_center[1]+300),(int(centralization.X_center),Robot_center[1]-300),(255,0,0),1)

        cv2.imshow(f'ROBOT_{counter_display}', frame2)
        cv2.moveWindow(f'ROBOT_{counter_display}', 50,500)

        if should_update == True:
            centralization.minimize_error(verbose=True)          
            
        if cv2.waitKey(10) & 0xFF == ord('q'):
            break   
    
    counter_display += 1
    cap.release()
    cv2.destroyAllWindows()


if __name__ ==  "__main__":
    model_path = '/home/ibraa04/catkin_ws/src/models/best-2.pt'
    
    centralization = Centralization_Processing()
    model = YOLO(model_path)
    centralize_omega()