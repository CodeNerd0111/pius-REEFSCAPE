import ntcore
import wpilib

class NetworkTablePublisher():
    def __init__(self) -> None:
        # Get the default instance of NetworkTables that was created automatically
        # when the robot program starts
        self.inst = ntcore.NetworkTableInstance.getDefault()
    
    def addNewTable(self, tableName:str) -> None:
        self.inst.getTable(tableName)

    def addNewTopic(self, tableName:str, topicName:str) -> None:
        self.inst.getTable(tableName).getTopic(topicName)

    def publishToTopic(self, tableName:str, topicName:str, value:any) -> None:
        self.inst.getTable(tableName).getTopic(topicName).genericPublish(value)