import cv2
import numpy as np
from yolov5_depend import detect_core
import json
# from ICP_depend import pose_core

class Algorithm:
    __name = "Algorithm"

    object_size ={
        'milk':{'length':53,'width':37,'height':130},
        'biscuit':{'length':53,'width':47,'height':208},
        'cake':{'length':104,'width':53,'height':235},
        'waffle':{'length':90,'width':38,'height':195},
        'lemon_tea':{'length':62,'width':40,'height':106},
        'tissue':{'length':93,'width':80,'height':155},
        'coke':{'length':58,'width':58,'height':147},
        'mushroom_soup':{'length':67,'width':67,'height':101},
        "tape":{'length':101,'width':101,'height':49},
    }

    def Name(self):
        name = "algorithm-py2"
        return name

    def Init(self):
        self.model = detect_core.model_init()
        return 1


    def Do(self, rgbImageData, rgbHeight, rgbWidth, rgbChan, objectID):

        detect_res = detect_core.run(rgbImageData,self.model)
        
        res_json = json.loads(detect_res)

        detected_items = [obj for obj in res_json if obj['object_id']==objectID]

        if detected_items:
            self.detection_status = True
            self.mistake = False
            self.objectID = objectID
            res = max(detected_items,key=lambda x:x['conf'])
        else:
            self.detection_status = False
            self.mistake = True
            return False
        
        self.center_X,self.center_Y = res['center'][0], res['center'][1]
        self.box_width = res['width']
        self.box_height = res['height']
        self.objectName = res['object']
        self.x1, self.y1, self.x2, self.y2 = self.YOLO2COCO(self.center_X, self.center_Y, self.box_width, self.box_height, rgbWidth, rgbHeight)
        
    def YOLO2COCO(self, center_X,center_Y,box_width,box_height,rgbWidth,rgbHeight):
        x_center = int(center_X * rgbWidth)
        y_center = int(center_Y * rgbHeight)
        box_width = int(box_width * rgbWidth)
        box_height = int(box_height * rgbHeight)

        # 计算 x1、y1、x2、y2
        x1 = x_center - box_width // 2
        y1 = y_center - box_height // 2
        x2 = x1 + box_width
        y2 = y1 + box_height

        return x1, y1, x2, y2

    def ErrorDetect(self):
        return 1
    
    def GetX1(self):
        return self.x1
    
    def GetX2(self):
        return self.x2
    
    def GetY1(self):
        return self.y1
    
    def GetY2(self):
        return self.y2
    
    def GetSizeX(self):
        return  self.object_size['self.objectName']['length']
    
    def GetSizeY(self):
        return self.object_size['self.objectName']['width']
    
    def GetSizeZ(self):
        return self.object_size['self.objectName']['height']
    
    def GetObjectName(self):
        return self.objectName

    def GetCenterX(self):
        return self.center_X
    
    def GetCenterY(self):
        return self.center_Y
    
    def GetBoxWidth(self):
        return self.box_width
    
    def GetBoxHeight(self):
        return self.box_height
    
    def GetDetection_status(self):
        return self.detection_status
    
    def GetMistake(self):
        return self.mistake
    
    def GetObjectID(self):
        return self.objectID

if __name__=='__main__':

    image = cv2.imread('/home/orin1/data/yolov5/data/images/test.jpg')

    algo = Algorithm()
    algo.Init()
    algo.Do(image,image.shape[0],image.shape[1],image.shape[2],objectID=1)
    print(algo.GetObjectName())
