from cscore import CameraServer as CS
import robotpy_apriltag as AprTag
import commands2
import numpy as np
from constants import CameraConstants as CamVals
import cv2

class CameraInterface(commands2.Subsystem):

    def __init__(self):
        CS.enableLogging()
        # Get the UsbCamera from CameraServer
        self.camera = CS.startAutomaticCapture()
        # Set the resolution
        self.camera.setResolution(CamVals.kImageWidth, CamVals.kImageHeight)
        # Get a CvSink. This will capture images from the camera
        self.cvSink = CS.getVideo()
        # Setup a CvSource. This will send images back to the Dashboard
        self.outputStream = CS.putVideo("Rectangle", CamVals.kImageWidth, CamVals.kImageHeight)

        # Setup an April Tag Detector
        self.detector = AprTag.AprilTagDetector()
        self.detector.addFamily("tag36h11", CamVals.kAprTagBitCorrectionMax)
        # Allocating new images is very expensive, always try to preallocate
        self.mat = np.zeros(shape=(CamVals.kImageHeight, CamVals.kImageWidth, 3), dtype=np.uint8)
        
    def getTagIDs(self):
        """Detects April Tags and returns a list of ID's"""
        
        image = self.cvSink.grabFrame(self.mat)[1]
        # Convert the frame to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Detect apriltag
        detections = self.detector.detect(gray)
        
        IDs = []
        for tag in detections:
            IDs.append(tag.getId())
        return IDs

        
        