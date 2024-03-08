import cv2
import numpy as np

class Algorithm:
    __name = "Algorithm"

    def Name(self):
        name = "algorithm-py0"
        return name

    def Init(self):
        return 1

    def Do(self, rgbImageData, rgbWidth, rgbHight, rgbChan, depthImageData, depWidth, depHight, depChan, objectID):
        # your function
        self.x = 1
        self.y = 1
        self.z = 1
        self.roll = 1
        self.pitch = 1
        self.yaw = 1
        self.sizeX = 1
        self.sizeY = 1
        self.recognition_status = 1
        self.mistake = 1
        self.ObjectID = 1

    def ErrorDetect(self):
        return 1


    def GetX(self):
        return self.x
    def GetY(self):
        return self.y
    def GetZ(self):
        return self.z
    def GetRoll(self):
        return self.roll
    def GetPitch(self):
        return self.pitch
    def GetYaw(self):
        return self.yaw
    def GetSizeX(self):
        return self.sizeX
    def GetSizeY(self):
        return self.sizeY
    def GetRecognition_status(self):
        return self.recognition_status
    def GetMistake(self):
        return self.mistake
    def GetObjectID(self):
        return self.ObjectID

