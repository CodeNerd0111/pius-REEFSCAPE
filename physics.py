
#
# See the documentation for more details on how this works
#
# Documentation can be found at https://robotpy.readthedocs.io/projects/pyfrc/en/latest/physics.html
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#
# Examples can be found at https://github.com/robotpy/examples

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import motor_cfgs, tankmodel
from pyfrc.physics.units import units
from pyfrc.physics.drivetrains import four_motor_swerve_drivetrain
from wpilib import simulation
from constants import ElevatorConstants as const
import wpilib
import typing
import wpilib.simulation
import wpimath.system.plant
if typing.TYPE_CHECKING:
    from robot import MyRobot


class PhysicsEngine:
    """
    Simulates a 4-wheel robot using Tank Drive joystick control
    """

    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        :param robot: your robot object
        """

        self.physics_controller = physics_controller
        deviceNames = simulation.SimDeviceSim.enumerateDevices()
        self.devices:list[simulation.SimDeviceSim] = (simulation.SimDeviceSim(str(name)) for name in deviceNames)
        self.motors:list[simulation.SimDeviceSim] = []
        for device in self.devices:
            if len(device.getName()) < 15:
                self.motors.append(device)
        

        bumper_width = 3.25 * units.inch

        self.drivetrain = tankmodel.TankModel.theory(
            motor_cfgs.MOTOR_CFG_CIM,           # motor configuration
            110 * units.lbs,                    # robot mass
            10.71,                              # drivetrain gear ratio \omega In/\omega Out
            2,                                  # motors per side
            22 * units.inch,                    # robot wheelbase
            23 * units.inch + bumper_width * 2, # robot width
            32 * units.inch + bumper_width * 2, # robot length
            6 * units.inch,                     # wheel diameter
        )
                # This gearbox represents a gearbox containing 4 Vex 775pro motors.
        self.elevatorGearbox = wpimath.system.plant.DCMotor.vex775Pro(4)

        # Simulation classes help us simulate what's going on, including gravity.
        self.elevatorSim = wpilib.simulation.ElevatorSim(
            self.elevatorGearbox,
            const.kElevatorGearing,
            const.kCarriageMass,
            const.kElevatorDrumRadius,
            const.kMinElevatorHeight,
            const.kMaxElevatorHeight,
            True,
            0,
            [0.01, 0.0],
        )
        self.encoderSim = wpilib.simulation.EncoderSim(wpilib.Encoder(1, 0))
        self.motorSim = wpilib.simulation.PWMSim(wpilib.PWMSparkMax(0))

        # Create a Mechanism2d display of an elevator
        self.mech2d = wpilib.Mechanism2d(20, 50)
        self.elevatorRoot = self.mech2d.getRoot("Elevator Root", 10, 0)
        self.elevatorMech2d = self.elevatorRoot.appendLigament(
            "Elevator", self.elevatorSim.getPositionInches(), 90
        )

        # Put Mechanism to SmartDashboard
        wpilib.SmartDashboard.putData("Elevator Sim", self.mech2d)

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        # Simulate the drivetrain
        # Front Left, Back Left, Front Right, Back Right - Original Order
        # Back Left, Back Right, Front Left, Front Right - Calculation Order

        motorPositions = tuple(motor.getDouble("Position").get() for motor in self.motors)
        motorSpeeds = tuple(motor.getDouble("Velocity").get() for motor in self.motors)
        chassisSpeeds = four_motor_swerve_drivetrain(motorSpeeds[1],
                                                     motorSpeeds[3],
                                                     motorSpeeds[0],
                                                     motorSpeeds[2],
                                                     motorPositions[5],
                                                     motorPositions[7],
                                                     motorPositions[4],
                                                     motorPositions[6]
                                                     )
        pose = self.physics_controller.drive(chassisSpeeds, tm_diff)

        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        # First, we set our "inputs" (voltages)
        self.elevatorSim.setInput(
            0, self.motorSim.getSpeed() * wpilib.RobotController.getInputVoltage()
        )

        # Next, we update it
        self.elevatorSim.update(tm_diff)

        # Finally, we set our simulated encoder's readings and simulated battery
        # voltage
        self.encoderSim.setDistance(self.elevatorSim.getPosition())
        # SimBattery estimates loaded battery voltage
        # wpilib.simulation.RoboRioSim.setVInVoltage(
        #     wpilib.simulation.BatterySim
        # )

        # Update the Elevator length based on the simulated elevator height
        self.elevatorMech2d.setLength(self.elevatorSim.getPositionInches())