from cscore import CameraServer as CS
import robotpy_apriltag as AprTag
import numpy as np
from constants import CameraConstants as CamVals
import cv2
import ntcore
from wpimath import geometry



def main():
    CS.enableLogging()
    tagPublisher = Packager()
    # Get the UsbCamera from CameraServer
    camera = CS.startAutomaticCapture()
    # Set the resolution
    camera.setResolution(CamVals.kImageWidth, CamVals.kImageHeight)
    camera.setBrightness(50)

    # Get a CvSink. This will capture images from the camera
    cvSink = CS.getVideo()
    # Setup a CvSource. This will send images back to the Dashboard
    outputStream = CS.putVideo("Default", CamVals.kImageWidth, CamVals.kImageHeight)

    grayScaleStream = CS.putVideo("GrayScale", CamVals.kImageWidth, CamVals.kImageHeight)

    # Setup an April Tag Detector
    detector = AprTag.AprilTagDetector()
    detector.addFamily("tag36h11", CamVals.kAprTagBitCorrectionMax)
    estimator = AprTag.AprilTagPoseEstimator(
        AprTag.AprilTagPoseEstimator.Config(CamVals.kAprilTagSize, 
                                            fx=CamVals.kHorizontalFocalLength, 
                                            fy=CamVals.kVerticalFocalLength, 
                                            cx=CamVals.kImageWidth/2, 
                                            cy=CamVals.kImageHeight/2))

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

        

        grayScaleMat = cv2.cvtColor(mat, cv2.COLOR_BGR2GRAY)
        # Detect all apriltags
        detections = detector.detect(grayScaleMat)

        # Put the data to NetworkTables
        tagPublisher.clear()
        for tag in detections:
            drawDetectionBox(tag, mat) # Draws to "Default"
            tagPose = estimator.estimate(tag)
            tagPublisher.addDetectedTag(tag, tagPose)
        
        tagPublisher.publishAllTags()

        # Give the output stream a new image to display (MUST COME AFTER ALL OTHER PROCESSING CODE)
        outputStream.putFrame(mat)
        grayScaleStream.putFrame(grayScaleMat)


def drawDetectionBox(tag:AprTag.AprilTagDetection, mat):
    """
    Draws a box around the tag

    :param tag: The tag to draw a box around
    :param mat: The image frame that this method draws to
    """
    bL, bR, tR, tL = tag.getCorner(0), tag.getCorner(1), tag.getCorner(2), tag.getCorner(3)

    cv2.line(mat, (int(bL.x), int(bL.y)), (int(bR.x), int(bR.y)), (0, 0, 0), 5)
    cv2.line(mat, (int(bR.x), int(bR.y)), (int(tR.x), int(tR.y)), (0, 255, 255), 5)
    cv2.line(mat, (int(tR.x), int(tR.y)), (int(tL.x), int(tL.y)), (255, 0, 255), 5)
    cv2.line(mat, (int(tL.x), int(tL.y)), (int(bL.x), int(bL.y)), (255, 255, 0), 5)


class Packager:
    def __init__(self):
        """A class to handle the publishing of all necessary April Tag data"""
        NT = ntcore.NetworkTableInstance.getDefault()
        tagTable = NT.getTable("AprilTag")
        self.publishers = [tagTable.getFloatArrayTopic("Centers_x").publish(),
                    tagTable.getFloatArrayTopic("Centers_y").publish(),
                    tagTable.getFloatArrayTopic("Positions_x").publish(),
                    tagTable.getFloatArrayTopic("Positions_y").publish(),
                    tagTable.getFloatArrayTopic("Positions_z").publish(),
                    tagTable.getFloatArrayTopic("Roll").publish(),
                    tagTable.getFloatArrayTopic("Pitch").publish(),
                    tagTable.getFloatArrayTopic("Yaw").publish()]
        
        self.tagList = dict(str, list[float])
        for publisher in self.publishers:
            publisher.setDefault([])
            self.tagList.setdefault(publisher.getTopic().getName(), [])

        
    def addDetectedTag(self, tag:AprTag.AprilTagDetection, tagPose:geometry.Transform3d):
        """
        Adds Tag Information to publishing cache

        :param tag: The tag detected
        :param tagPose: The estimated 3D pose of the tag in space
        """
        self.tagList.get("Centers_x").append(tag.getCenter().x)
        self.tagList.get("Centers_y").append(tag.getCenter().y)
        self.tagList.get("Positions_x").append(tagPose.x)
        self.tagList.get("Positions_y").append(tagPose.y)
        self.tagList.get("Positions_z").append(tagPose.z)
        self.tagList.get("Roll").append(tagPose.rotation().x)
        self.tagList.get("Pitch").append(tagPose.rotation().y)
        self.tagList.get("Yaw").append(tagPose.rotation().z)
        tagPose.rotation().y
    
    def publishAllTags(self):
        """
        Publishes all tag information in the cache
        """
        for publisher in self.publishers:
            publisher.set(self.tagList.get(publisher.getTopic().getName()))
    
    def clear(self):
        """
        Clears published values and publishing cache
        """
        for publisher in self.publishers:
            publisher.setDefault([])
            self.tagList.setdefault(publisher.getTopic().getName(), [])

