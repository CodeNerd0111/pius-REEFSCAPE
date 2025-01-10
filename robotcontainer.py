import commands2
import commands2.cmd
import commands2.button

import constants

import subsystems.drivesubsystem
import subsystems.camerasubsystem
import commands.drivedistanceprofiled

import wpimath.trajectory


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.

    """

    def __init__(self):
        # The robot's subsystems need to be declared here:
        self.robotDrive = subsystems.drivesubsystem.DriveSubsystem()
        self.cameraInfo = subsystems.camerasubsystem.CameraInterface()

        # Retained command references
        self.driveFullSpeed = commands2.cmd.runOnce(
            lambda: self.robotDrive.setMaxOutput(1), self.robotDrive
        )
        self.driveHalfSpeed = commands2.cmd.runOnce(
            lambda: self.robotDrive.setMaxOutput(0.5), self.robotDrive
        )

        # The driver's controllers
        self.XBoxController = commands2.button.CommandXboxController(
            constants.OIConstants.kXBoxControllerPort
        )
        self.leftJoystick = commands2.button.CommandJoystick(
            constants.OIConstants.kLeftJoystickPort
        )
        self.rightJoystick = commands2.button.CommandJoystick(
            constants.OIConstants.kRightJoystickPort
        )

        # Configure the button bindings
        self.configureButtonBindings()

        # Configure default commands
        # Set the default drive command to split-stick arcade drive
        self.robotDrive.setDefaultCommand(
            # A split-stick arcade command, with forward/backward controlled by the left
            # hand, and turning controlled by the right.
            commands2.cmd.run(
                lambda: self.robotDrive.arcadeDrive(
                    -self.leftJoystick.getY(),
                    -self.leftJoystick.getX(),
                ),
                self.robotDrive
            )
        )
        

    def configureButtonBindings(self):
        """
        Use this method to define your button->command mappings. Buttons can be created via the button
        factories on commands2.button.CommandGenericHID or one of its
        subclasses (commands2.button.CommandJoystick or command2.button.CommandXboxController).
        """

        # Configure your button bindings here

        # We can bind commands while retaining references to them in RobotContainer

        # Drive at half speed when the bumper is held
        self.XBoxController.rightBumper().onTrue(self.driveHalfSpeed).onFalse(
            self.driveFullSpeed
        )

        # Drive forward by 3 meters when the 'A' button is pressed, with a timeout of 10 seconds
        self.XBoxController.a().onTrue(
            commands.drivedistanceprofiled.DriveDistanceProfiled(
                3, self.robotDrive
            ).withTimeout(10)
        )

        # Do the same thing as above when the 'B' button is pressed, but defined inline
        self.XBoxController.b().onTrue(
            commands2.TrapezoidProfileCommand(
                wpimath.trajectory.TrapezoidProfile(
                    # Limit the max acceleration and velocity
                    wpimath.trajectory.TrapezoidProfile.Constraints(
                        constants.DriveConstants.kMaxSpeedMetersPerSecond,
                        constants.DriveConstants.kMaxAccelerationMetersPerSecondSquared,
                    ),
                ),
                # Pipe the profile state to the drive - or in english, tell the robotdrive what to do based on setpointState
                lambda setpointState: self.robotDrive.setDriveStates(
                    setpointState, setpointState
                ),
                # End at desired position in meters; implicitly starts at 0
                lambda: wpimath.trajectory.TrapezoidProfile.State(3, 0), # The profile state where we want to be (3 meters ahead)
                wpimath.trajectory.TrapezoidProfile.State, # The profile state where we are now
                self.robotDrive, # We will need robotDrive for this command
            )
            .beforeStarting(self.robotDrive.resetEncoders)
            .withTimeout(10)
        )
        






    def getAutonomousCommand(self) -> commands2.Command:
        """
        Use this to pass the autonomous command to the main :class:`.Robot` class.

        :returns: the command to run in autonomous
        """
        return commands2.cmd.none()