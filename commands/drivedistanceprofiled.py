import commands2

import wpimath.trajectory
import subsystems.drivesubsystem
import constants


class DriveDistanceProfiled(commands2.TrapezoidProfileCommand):
    """Drives a set distance using a motion profile."""

    def __init__(
        self, meters: float, drive: subsystems.drivesubsystem.DriveSubsystem
    ) -> None:
        """Creates a new DriveDistanceProfiled command.

        :param meters: The distance to drive.
        :param drive:  The drive subsystem to use.
        """

        # super().__init__() calls the initialization method of this class's parent. 
        # We are basically rewriting TrapezoidProfileCommand from commands2 to connect it to our driveState and fit our needs.
        super().__init__( 
            wpimath.trajectory.TrapezoidProfile(
                # Limit the max acceleration and velocity
                wpimath.trajectory.TrapezoidProfile.Constraints(
                    constants.DriveConstants.maxVelocity,
                    constants.DriveConstants.maxAcceleration,
                )
            ),
            # Pipe the profile state to the drive
            lambda setpointState: drive.desire_module_states([setpointState] * 4),
            # End at desired position in meters; implicitly starts at 0
            lambda: wpimath.trajectory.TrapezoidProfile.State(meters, 0),
            # Current position
            lambda: wpimath.trajectory.TrapezoidProfile.State(0, 0),
            # Require the drive
            drive,
        )
        # Reset drive encoders since we're starting at 0
        drive.reset_modules()

'''
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

            '''