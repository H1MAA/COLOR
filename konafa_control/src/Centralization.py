import os
import sys

parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(parent_dir)
from control import Control
from PID import PID_class

class Centralization_Processing():
    def __init__(self):
        self.frame_width = 0
        self.frame_height = 0
        self.speeds = [0]*2
        self.speedsPrev = [1]*2
        self.Lateral_error = 0
        self.Lateral_error_normalized = 0.5
        self.PIDlateral = PID_class(5,0,0)        
        self.isCentered_lat = False
        self.center_lat_frames = 0
        self.X_center = None
        self.Robot_center = None
        self.cntrl = Control()

    def Update_errors(self,result):

        boxes_raw = result[0].boxes.xywh.cpu().numpy()
        if len(boxes_raw) == 0:
            return False
        
        baloon1 = boxes_raw[0]
        x = baloon1[0]
        y = baloon1[1]
        w = baloon1[2]
        h = baloon1[3]
        area = w*h

        self.Lateral_error = (int(x) - self.Robot_center[0])
        self.Lateral_error_normalized = self.Lateral_error/w
        self.X_center = x
        return True

    def moveAdjust(self,indx,error):
        try:
            if error < 0.07 and error > -0.07:
                self.speeds[indx] = 0
                return True

            if indx == 0:
                omegaPID = self.PIDlateral.calculate(self.Lateral_error_normalized)
                self.speeds[0] = -omegaPID

            return False
        
        except Exception as e:
            print(f"ERROR: {e}")
            return False

    def minimize_error(self,verbose = False):
       
        if self.isCentered_lat:
            self.center_lat_frames +=1
        else: 
            self.center_lat_frames = 0 

        if self.center_lat_frames  < 50:
            self.isCentered_lat = self.moveAdjust(0,self.Lateral_error_normalized)
        else:
            exit(0)

        if verbose:
            print(f"Lateral Error: {self.Lateral_error_normalized}\t")
            print(f"isCentered_lat: {self.isCentered_lat}\t")
    
        if self.speeds != self.speedsPrev:
            self.cntrl.set_omega(self.speeds[0])
            

        self.speedsPrev = self.speeds.copy()
        return self.isCentered_lat