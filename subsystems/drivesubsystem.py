import wpilib
import wpilib.drive
import commands2
import wpimath.controller
import wpimath.trajectory

import constants
import smartmotorcontroller

# NOTE: We will initialize all of our motors in this subsystem file
# Currently, a quad-motor example is being used

class DriveSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        """Creates a new DriveSubsystem"""
        super().__init__()
        # The motors on the left side of the drive. - Main
        self.leftLeader = smartmotorcontroller.SmartMotorController(
            constants.DriveConstants.kLeftMotor1Port
        )
        # Follows the main left motor so both can be passed into the Differential Drive
        self.leftFollower = smartmotorcontroller.SmartMotorController(
            constants.DriveConstants.kLeftMotor2Port
        )

        # The motors on the right side of the drive. - Main
        self.rightLeader = smartmotorcontroller.SmartMotorController(
            constants.DriveConstants.kRightMotor1Port
        )
        # Follows the main right motor so both can be passed into the Differential Drive
        self.rightFollower = smartmotorcontroller.SmartMotorController(
            constants.DriveConstants.kRightMotor2Port
        )

        # We need to invert one side of the drivetrain so that positive voltages
        # result in both sides moving forward. Depending on how your robot's
        # gearbox is constructed, you might have to invert the left side instead.
        # self.rightLeader.setInverted(True)

        # You might need to not do this depending on the specific motor controller
        # that you are using -- contact the respective vendor's documentation for
        # more details.
        # self.rightFollower.setInverted(True)

        self.leftFollower.follow(self.leftLeader)
        self.rightFollower.follow(self.rightLeader)

        self.leftLeader.setPID(constants.DriveConstants.kp, 0, 0)
        self.rightLeader.setPID(constants.DriveConstants.kp, 0, 0)

        # The feedforward controller (note that these are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!)
        # check DriveConstants for more information.
        self.feedforward = wpimath.controller.SimpleMotorFeedforwardMeters(
            constants.DriveConstants.ksVolts,
            constants.DriveConstants.kvVoltSecondsPerMeter,
            constants.DriveConstants.kMaxAccelerationMetersPerSecondSquared,
        )

        # The robot's drive
        self.drive = wpilib.drive.DifferentialDrive(self.leftLeader, self.rightLeader)

    def arcadeDrive(self, fwd: float, rot: float):
        """
        Drives the robot using arcade controls.

        :param fwd: the commanded forward movement
        :param rot: the commanded rotation
        """
        self.drive.arcadeDrive(fwd, rot)

    def setDriveStates(
        self,
        left: wpimath.trajectory.TrapezoidProfile.State,
        right: wpimath.trajectory.TrapezoidProfile.State,
    ):
        """
        Attempts to follow the given drive states using offboard PID.

        :param left:  The left wheel state.
        :param right: The right wheel state.
        """
        self.leftLeader.setSetPoint(
            smartmotorcontroller.SmartMotorController.PIDMode.kPosition,
            left.position,
            self.feedforward.calculate(left.velocity),
        )

        self.rightLeader.setSetPoint(
            smartmotorcontroller.SmartMotorController.PIDMode.kPosition,
            right.position,
            self.feedforward.calculate(right.velocity),
        )

    def getLeftEncoderDistance(self) -> float:
        """Returns the left drive encoder distance."""
        return self.leftLeader.getEncoderDistance()

    def getRightEncoderDistance(self) -> float:
        """Returns the right drive encoder distance."""
        return self.rightLeader.getEncoderDistance()

    def resetEncoders(self):
        """Resets the drive encoders"""
        self.leftLeader.resetEncoder()
        self.rightLeader.resetEncoder()

    def setMaxOutput(self, maxOutput: float):
        """
        Sets the max output of the drive. Useful for scaling the drive to drive more slowly.

        :param maxOutput: the maximum output to which the drive will be constrained
        """
        self.drive.setMaxOutput(maxOutput) # maxOutput is multiplied to the motor power value, so a value of 1 is normal.