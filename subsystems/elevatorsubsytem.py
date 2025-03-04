import wpilib
import commands2
from wpimath.trajectory import TrapezoidProfile
from constants import ElevatorConstants as elv
from rev import SparkMax
from wpimath.controller import ElevatorFeedforward

class ElevatorSubsytem(commands2.subsystem):
    
    def __init__(self):
        """"
        Creates a Elevator subsystem
        """
        self._firstStage = SparkMax(elv.fS_motorPort, SparkMax.MotorType.kBrushless)
        self._secondstage = SparkMax(elv.sS_motorPort, SparkMax.MotorType.kBrushless)
        self._thirdstage = SparkMax(elv.tS_motorPort, SparkMax.MotorType.kBrushless)

        self._feedforward = ElevatorFeedforward(elv.kS, elv.kG, elv.kV, elv.kA)

    def elevate(self, speed:float):
        """
        The module follows a fixed velocity. Negative velocities will move the module
        down, and positive velocities will move the module up.
        
        :param speed: The speed that the module will travel, in inches/sec.

        """
        pass

    def set_desired_height(self, height:float):
        """
        Sets a goal height for the elevator module.

        :param: The height to go to, in inches

        """
        pass
    

