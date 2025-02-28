import commands2
import commands2.cmd
import commands2.button

import constants
import ntcore

import subsystems.drivesubsystem

import visionprocessing.apriltagpackager as ATPackage
from wpimath import geometry
import wpilib
from pathplannerlib.path import PathPlannerPath




# TODO: Subsystems list
# 1. Swerve module
# 2. Elevator Module
# 3. Manipulator
# ...


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
        wpilib.CameraServer.launch("vision.py:main")


        # Set up NetworkTables
        self.publisher = ntcore.NetworkTableInstance.getDefault()

        # Set up Autonomous Controls
        self.autoChooser = wpilib.SendableChooser()
        self.autoChooser.addOption("Go to Cage Chor Cmd", 1)


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
        self.configureTriggerCommands()
        self.configureButtonBindings()

        # Configure default commands
        # Set the default drive command to split-stick arcade drive
        self.robotDrive.setDefaultCommand(
            # A split-stick arcade command, with forward/backward controlled by the left
            # hand, and turning controlled by the right.
            commands2.cmd.run(
                lambda: self.robotDrive.drive(
                    geometry.Translation2d(
                        -self.leftJoystick.getX(),
                        -self.leftJoystick.getY()),
                    self.rightJoystick.getY(),
                    True, True
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
        pass

    
    def configureTriggerCommands(self) -> None:
        """
        Use this method to define your Trigger->command mappings. 
        """

        # Example Trigger
        aprilTagUnpacker = ATPackage.AprilTagUnpacker()
        self.tagsDetected = commands2.button.Trigger(
            lambda: len(aprilTagUnpacker.getTags()) > 0
        )


    def getAutonomousCommand(self) -> commands2.Command:
        """
        Use this to pass the autonomous command to the main :class:`.Robot` class.

        :returns: the command to run in autonomous
        """
        return self.robotDrive.follow_trajectory_command(
            PathPlannerPath.fromChoreoTrajectory("goToCage"), 
            self.robotDrive.TrajectoryFollowerParameters,
            self.robotDrive.RobotConfigControls.config,
            True,
            True 
            )



        