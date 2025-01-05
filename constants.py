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