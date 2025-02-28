import swervepy.subsystem
import swervepy.impl
from constants import DriveConstants as dc
from constants import RobotConfigControls as rcc
from wpimath.geometry import Rotation2d, Translation2d
from wpimath.system.plant import DCMotor
from pint import Quantity
from pathplannerlib.config import RobotConfig, PIDConstants, ModuleConfig

class DriveSubsystem(swervepy.subsystem.SwerveDrive):

    def __init__(self):
        """"
        Creates a swerve drive subsystem
        """
        # Drive Components
        m_frontLeft = swervepy.impl.NEOCoaxialDriveComponent(dc.fL_MotorPort, dc.drive_params)
        m_backLeft = swervepy.impl.NEOCoaxialDriveComponent(dc.bL_MotorPort, dc.drive_params)
        m_frontRight = swervepy.impl.NEOCoaxialDriveComponent(dc.fR_MotorPort, dc.drive_params)
        m_backRight = swervepy.impl.NEOCoaxialDriveComponent(dc.bR_MotorPort, dc.drive_params)

        # Encoders
        enc_frontLeft = swervepy.impl.AbsoluteCANCoder(dc.fL_EncoderPort)
        enc_backLeft = swervepy.impl.AbsoluteCANCoder(dc.bL_EncoderPort)
        enc_frontRight = swervepy.impl.AbsoluteCANCoder(dc.fR_EncoderPort)
        enc_backRight = swervepy.impl.AbsoluteCANCoder(dc.bR_EncoderPort)

        # Azimuth module offset. This is the value reported by the absolute encoder when the wheel is pointed straight.
        offset = Rotation2d.fromDegrees(0)
        # Azimuth Components
        a_frontLeft = swervepy.impl.NEOCoaxialAzimuthComponent(dc.fL_AzimuthPort, offset, dc.azimuth_params, enc_frontLeft)
        a_backLeft = swervepy.impl.NEOCoaxialAzimuthComponent(dc.bL_AzimuthPort, offset, dc.azimuth_params, enc_backLeft)
        a_frontRight = swervepy.impl.NEOCoaxialAzimuthComponent(dc.fR_AzimuthPort, offset, dc.azimuth_params, enc_frontRight)
        a_backRight = swervepy.impl.NEOCoaxialAzimuthComponent(dc.bR_AzimuthPort, offset, dc.azimuth_params, enc_backRight)

        # Swerve Modules
        frontLeft = swervepy.impl.CoaxialSwerveModule(
            m_frontLeft, a_frontLeft, Translation2d(dc.wheelBase / 2, dc.trackWidth / 2))
        backLeft = swervepy.impl.CoaxialSwerveModule(
            m_backLeft, a_backLeft, Translation2d(-dc.wheelBase / 2, dc.trackWidth / 2))
        frontRight = swervepy.impl.CoaxialSwerveModule(
            m_frontRight, a_frontRight, Translation2d(dc.wheelBase / 2, -dc.trackWidth / 2))
        backRight = swervepy.impl.CoaxialSwerveModule(
            m_backRight, a_backRight, Translation2d(-dc.wheelBase / 2, -dc.trackWidth / 2))
        
        gyro = swervepy.impl.NAVXGyro() # Replace with real gyro

        super().__init__(
            [frontLeft, backLeft, frontRight, backRight],
            gyro,
            dc.maxVelocity,
            dc.maxAngularVelocity
        )
    
    class TrajectoryFollowerParameters:
        max_drive_velocity = dc.maxVelocity

        # Positional PID constants for X, Y, and theta (rotation) controllers
        theta_kP = dc.azimuth_params.kP
        xy_kP = dc.drive_params.kP

    class RobotConfigControls:
        massKG: float = rcc.massKG
        MOI: float = rcc.MOI
        moduleOffsets = rcc.moduleOffsets
        wheelRadiusMeters: float = rcc.wheelRadiusMeters
        maxDriveVelocityMPS: float = rcc.maxDriveVelocityMPS
        wheelCOF: float = rcc.wheelCOF
        driveCurrentLimit: float = rcc.driveCurrentLimit
        numMotors: int = rcc.numMotors

        nominalVoltage: Quantity = rcc.nominalVoltage # volts
        stallTorque: Quantity = rcc.stallTorque # newton_meters
        stallCurrent: Quantity = rcc.stallCurrent # amperes
        freeCurrent: Quantity = rcc.freeCurrent # amperes
        freeSpeed: Quantity = rcc.freeSpeed # radians_per_second
        numMotors: int = rcc.numMotors
        
        driveMotor = DCMotor(nominalVoltage, stallTorque, stallCurrent, freeCurrent, freeSpeed, numMotors)

        config = RobotConfig(massKG, MOI, 
                             ModuleConfig(wheelRadiusMeters, maxDriveVelocityMPS, wheelCOF, driveMotor, driveCurrentLimit, numMotors),
                             moduleOffsets)