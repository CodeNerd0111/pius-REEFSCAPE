import math


class DriveConstants:
    kLeftMotor1Port = 0
    kLeftMotor2Port = 1
    kRightMotor1Port = 2
    kRightMotor2Port = 3

    ksVolts = 1
    kvVoltSecondsPerMeter = 0.8
    kaVoltSecondsSquaredPerMeter = 0.15

    kp = 1

    kMaxSpeedMetersPerSecond = 3
    kMaxAccelerationMetersPerSecondSquared = 3


class OIConstants:
    kXBoxControllerPort = 2
    kLeftJoystickPort = 0
    kRightJoystickPort = 1

class CameraConstants:
    kImageWidth = 640
    kImageHeight = 480
    kAprTagBitCorrectionMax = 2
    kAprilTagSize = 0.1524
    kVerticalFocalLength = 700
    kHorizontalFocalLength = 700