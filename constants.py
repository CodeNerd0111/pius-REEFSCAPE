import math
from swervepy import u
import swervepy.impl
from wpimath.geometry import Translation2d
from wpimath import units

class DriveConstants:
    drive_params = swervepy.impl.TypicalDriveComponentParameters(
        wheel_circumference=4 * math.pi * u.inch,
        gear_ratio=6.75 / 1,  # SDS Mk4i L2
        max_speed=4.5 * (u.m / u.s),
        open_loop_ramp_rate=0,
        closed_loop_ramp_rate=0,
        continuous_current_limit=40,
        peak_current_limit=60,
        peak_current_duration=0.01,
        neutral_mode=swervepy.impl.NeutralMode.COAST,
        kP=0.8,
        kI=0.3,
        kD=0.4,
        kS=0,
        kV=0,
        kA=0,
        invert_motor=False,
    )
    azimuth_params = swervepy.impl.TypicalAzimuthComponentParameters(
        gear_ratio=150 / 7,  # SDS Mk4i
        max_angular_velocity=11.5 * (u.rad / u.s),
        ramp_rate=0,
        continuous_current_limit=40,
        peak_current_limit=60,
        peak_current_duration=0.01,
        neutral_mode=swervepy.impl.NeutralMode.BRAKE,
        kP=0.3,
        kI=0.1,
        kD=0.2,
        invert_motor=False,
    )

    fL_MotorPort = 9
    bL_MotorPort = 10
    fR_MotorPort = 7
    bR_MotorPort = 5
    fL_AzimuthPort = 3
    bL_AzimuthPort = 4
    fR_AzimuthPort = 6
    bR_AzimuthPort = 2
    fL_EncoderPort = 13
    bL_EncoderPort = 11
    fR_EncoderPort = 14
    bR_EncoderPort = 12

    wheelBase = 0.05
    trackWidth = 1.8

    maxVelocity = 5 * (u.m / u.s)
    maxAcceleration = 6 * (u.m / u.s / u.s)
    maxAngularVelocity = 3 * (u.rad / u.s)
    maxAngularAcceleration = 5 * (u.rad / u.s / u.s)

class RobotConfigControls:
    massKG: float = 70
    MOI: float = 6
    moduleOffsets = [
        Translation2d(DriveConstants.wheelBase / 2, DriveConstants.trackWidth / 2),
        Translation2d(DriveConstants.wheelBase / 2, -DriveConstants.trackWidth / 2),
        Translation2d(-DriveConstants.wheelBase / 2, DriveConstants.trackWidth / 2),
        Translation2d(-DriveConstants.wheelBase / 2, -DriveConstants.trackWidth / 2)
    ]
    wheelRadiusMeters: float = 0.05
    maxDriveVelocityMPS: float = 5
    wheelCOF: float = 1.5
    driveCurrentLimit: float = 2
    numMotors: int = 1

    nominalVoltage: units.volts = 12
    stallTorque: units.newton_meters = 3.75
    stallCurrent: units.amperes = 150
    freeCurrent: units.amperes = 1.8
    freeSpeed: units.radians_per_second = 594.3893
    numMotors: int = 1

class OIConstants:
    kXBoxControllerPort = 2
    kLeftJoystickPort = 0
    kRightJoystickPort = 1
    kDeadZone = 0.1

class CameraConstants:
    kImageWidth = 640
    kImageHeight = 480
    kAprTagBitCorrectionMax = 2
    kAprilTagSize = 0.1524
    kVerticalFocalLength = 700
    kHorizontalFocalLength = 700

class ElevatorConstants:
    # Place Holder numbers
    kElevatorKp = 5.0
    kElevatorGearing = 10.0
    kElevatorDrumRadius = 0.0508  
    kCarriageMass = 4

    kMinElevatorHeight = 0.0508  
    kMaxElevatorHeight = 1.27  
    kVel = 1.75
    kAcc = .75

    fS_motorPort = 1
    sS_motorPort = 2
    tS_motorPort = 3

    kS: units.volts =  0
    kG: units.volts = 0
    kV: units.volt_seconds_per_meter = 0
    kA: units.volt_seconds_squared_per_meter = 0