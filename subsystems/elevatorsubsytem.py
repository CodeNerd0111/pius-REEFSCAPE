import wpilib
import commands2
from wpimath.trajectory import TrapezoidProfile
from constants import ElevatorConstants as elv
from constants import OIConstants as oic

class ElevatorSubsytem(commands2.subsystem):
    
    def __init__(self):
        """"
        Creates a Elevator subsystem

        Introduce the motors per stage
        firststage = wpilib.talon(elv.fS_motorPort)
        secondstage = wpilib.(elv.sS_motorPort)
        thirdstage = wpilib.(elv.tS_motorPort)
        """
        pass
    

