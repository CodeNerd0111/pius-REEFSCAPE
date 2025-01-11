from cscore import CameraServer as CS
import robotpy_apriltag as AprTag
import numpy as np
from constants import CameraConstants as CamVals
import cv2
import ntcore


def main(self):
    CS.enableLogging()
    print("vision.py has been executed")

    publisher = ntcore.NetworkTableInstance.getDefault()
    pub_tagIDs = publisher.getTable("AprilTag").getIntegerArrayTopic("IDs").publish()

    # Get the UsbCamera from CameraServer
    camera = CS.startAutomaticCapture()
    # Set the resolution
    camera.setResolution(CamVals.kImageWidth, CamVals.kImageHeight)
    # Get a CvSink. This will capture images from the camera
    cvSink = CS.getVideo()
    # Setup a CvSource. This will send images back to the Dashboard
    outputStream = CS.putVideo("Rectangle", CamVals.kImageWidth, CamVals.kImageHeight)

    # Setup an April Tag Detector
    detector = AprTag.AprilTagDetector()
    detector.addFamily("tag36h11", CamVals.kAprTagBitCorrectionMax)
    # Allocating new images is very expensive, always try to preallocate
    mat = np.zeros(shape=(CamVals.kImageHeight, CamVals.kImageWidth, 3), dtype=np.uint8)

    while True:
        # Tell the CvSink to grab a frame from the camera and put it
        # in the source image.  If there is an error notify the output.
        time, mat = cvSink.grabFrame(mat)
        if time == 0:
            # Send the output the error.
            outputStream.notifyError(cvSink.getError())
            # skip the rest of the current iteration
            continue

        # Put a rectangle on the image
        cv2.rectangle(mat, (100, 100), (400, 400), (255, 255, 255), 5)

        # Give the output stream a new image to display
        outputStream.putFrame(mat)


        # Put the data to NetworkTables

        grayScaleMat = cv2.cvtColor(mat, cv2.COLOR_BGR2GRAY)
        detections = detector.detect(grayScaleMat)

        IDs = []
        for tag in detections:
            IDs.append(tag.getId())
        pub_tagIDs.set(IDs)

    
