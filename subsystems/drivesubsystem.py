import swervepy.subsystem
import swervepy.impl
from constants import DriveConstants as c
from wpimath.geometry import Rotation2d, Translation2d
import wpimath.trajectory

class DriveSubsystem(swervepy.subsystem.SwerveDrive):

    def __init__(self):
        """"
        Creates a swerve drive subsystem
        """
        # Drive Components
        m_frontLeft = swervepy.impl.NEOCoaxialDriveComponent(c.fL_MotorPort, c.drive_params)
        m_backLeft = swervepy.impl.NEOCoaxialDriveComponent(c.bL_MotorPort, c.drive_params)
        m_frontRight = swervepy.impl.NEOCoaxialDriveComponent(c.fR_MotorPort, c.drive_params)
        m_backRight = swervepy.impl.NEOCoaxialDriveComponent(c.bR_MotorPort, c.drive_params)

        # Encoders
        enc_frontLeft = swervepy.impl.AbsoluteCANCoder(c.fL_EncoderPort)
        enc_backLeft = swervepy.impl.AbsoluteCANCoder(c.bL_EncoderPort)
        enc_frontRight = swervepy.impl.AbsoluteCANCoder(c.fR_EncoderPort)
        enc_backRight = swervepy.impl.AbsoluteCANCoder(c.bR_EncoderPort)

        # Azimuth module offset. This is the value reported by the absolute encoder when the wheel is pointed straight.
        offset = Rotation2d.fromDegrees(0)
        # Azimuth Components
        a_frontLeft = swervepy.impl.NEOCoaxialAzimuthComponent(c.fL_AzimuthPort, offset, c.azimuth_params, enc_frontLeft)
        a_backLeft = swervepy.impl.NEOCoaxialAzimuthComponent(c.bL_AzimuthPort, offset, c.azimuth_params, enc_backLeft)
        a_frontRight = swervepy.impl.NEOCoaxialAzimuthComponent(c.fR_AzimuthPort, offset, c.azimuth_params, enc_frontRight)
        a_backRight = swervepy.impl.NEOCoaxialAzimuthComponent(c.bR_AzimuthPort, offset, c.azimuth_params, enc_backRight)

        # Swerve Modules
        frontLeft = swervepy.impl.CoaxialSwerveModule(
            m_frontLeft, a_frontLeft, Translation2d(c.wheelBase / 2, c.trackWidth / 2))
        backLeft = swervepy.impl.CoaxialSwerveModule(
            m_backLeft, a_backLeft, Translation2d(-c.wheelBase / 2, c.trackWidth / 2))
        frontRight = swervepy.impl.CoaxialSwerveModule(
            m_frontRight, a_frontRight, Translation2d(c.wheelBase / 2, -c.trackWidth / 2))
        backRight = swervepy.impl.CoaxialSwerveModule(
            m_backRight, a_backRight, Translation2d(-c.wheelBase / 2, -c.trackWidth / 2))
        
        gyro = swervepy.impl.DummyGyro() # Replace with real gyro

        super().__init__(
            [frontLeft, backLeft, frontRight, backRight],
            gyro,
            c.maxVelocity,
            c.maxAngularVelocity
        )
    
    class TrajectoryFollowerParameters:
        max_drive_velocity = c.maxVelocity

        # Positional PID constants for X, Y, and theta (rotation) controllers
        theta_kP = c.azimuth_params.kP
        xy_kP = c.drive_params.kP