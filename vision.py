from cscore import CameraServer as CS
import robotpy_apriltag as AprTag
import numpy as np
from constants import CameraConstants as CamVals
import cv2
import ntcore
from wpimath import geometry



def main():
    CS.enableLogging()
    publisher = ntcore.NetworkTableInstance.getDefault()
    AprilTagPublisher = [publisher.getTable("AprilTag").getFloatArrayTopic("Center").publish(),
                    publisher.getTable("AprilTag").getFloatArrayTopic("Homography").publish(),
                    publisher.getTable("AprilTag").getFloatArrayTopic("Position").publish(),
                    publisher.getTable("AprilTag").getFloatArrayTopic("Position [ft]").publish()]


    # Get the UsbCamera from CameraServer
    camera = CS.startAutomaticCapture()
    # Set the resolution
    camera.setResolution(CamVals.kImageWidth, CamVals.kImageHeight)
    camera.setBrightness(50)

    # Get a CvSink. This will capture images from the camera
    cvSink = CS.getVideo()
    # Setup a CvSource. This will send images back to the Dashboard
    outputStream = CS.putVideo("Rectangle", CamVals.kImageWidth, CamVals.kImageHeight)

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

        

        # Put the data to NetworkTables
        grayScaleMat = cv2.cvtColor(mat, cv2.COLOR_BGR2GRAY)
        detections = detector.detect(grayScaleMat)

        
        for tag in detections:
            drawDetectionBox(tag, mat)
            tagPose = estimator.estimate(tag)
            publishTagInfo(tag, tagPose, AprilTagPublisher)



        # Give the output stream a new image to display (MUST COME AFTER ALL OTHER PROCESSING CODE)
        outputStream.putFrame(mat)
        grayScaleStream.putFrame(grayScaleMat)

    
def publishTagInfo(tag:AprTag.AprilTagDetection, tagPose:geometry.Transform3d, publishers:list[ntcore.Publisher]):
    publishers[0].set([tag.getCenter().x, tag.getCenter().y])
    homography = []
    for val in tag.getHomography():
        homography.append(val)
    publishers[1].set(homography)
    publishers[2].set([tagPose.x, tagPose.y, tagPose.z])
    publishers[3].set([tagPose.x_feet, tagPose.y_feet, tagPose.z_feet])

def resetPublisherInfo(publishers:list[ntcore.Publisher]):
    for publisher in publishers:
        publisher.set([])

def drawDetectionBox(tag:AprTag.AprilTagDetection, mat):
    bL, bR, tR, tL = tag.getCorner(0), tag.getCorner(1), tag.getCorner(2), tag.getCorner(3)

    cv2.line(mat, (int(bL.x), int(bL.y)), (int(bR.x), int(bR.y)), (0, 0, 0), 5)
    cv2.line(mat, (int(bR.x), int(bR.y)), (int(tR.x), int(tR.y)), (0, 255, 255), 5)
    cv2.line(mat, (int(tR.x), int(tR.y)), (int(tL.x), int(tL.y)), (255, 0, 255), 5)
    cv2.line(mat, (int(tL.x), int(tL.y)), (int(bL.x), int(bL.y)), (255, 255, 0), 5)
    
