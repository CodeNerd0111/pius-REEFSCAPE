import ntcore
from wpimath import geometry



class _AprilTag:
    def __init__(self, id, cX, cY, tX, tY, tZ, roll, pitch, yaw):
        """
        Creates an April Tag object that encapsulates information about the tag
        
        :param id: The ID of the April Tag
        :param cX: The horizontal pixel coordinate in the frame
        :param cY: The vertical pixel coordinate in the frame
        :param tX: The horizontal displacement of the physical tag [meters]
        :param tY: The vertical displacement of the physical tag [meters]
        :param tZ: The profile displacement of the physical tag [meters]
        :param roll: The roll rotation of the physical tag [radians]
        :param pitch: The pitch rotation of the physical tag [radians]
        :param yaw: The yaw rotation of the physical tag [radians]
        
        """
        self.id = id
        self.center = (cX, cY)
        self.pose = geometry.Transform3d(tX, tY, tZ, geometry.Rotation3d(roll, pitch, yaw))

    def getPose(self) -> geometry.Transform3d:
        return self.pose

    def getX(self) -> float:
        return self.pose.x
    def getY(self) -> float:
        return self.pose.y
    def getZ(self) -> float:
        return self.pose.z
    def getXinFeet(self) -> float:
        return self.pose.x_feet
    def getYinFeet(self) -> float:
        return self.pose.y_feet
    def getZinFeet(self) -> float:
        return self.pose.z_feet



class AprilTagUnpacker:
    def __init__(self):
        NT = ntcore.NetworkTableInstance.getDefault()
        self.tagTable = NT.getTable("AprilTag")

    def getTags(self) -> list[_AprilTag]:
        tagList = []
        cX, cY, tX, tY, tZ, roll, pitch, yaw = self.tagTable.getFloatArrayTopic("Centers_x").getEntry(), 
        self.tagTable.getFloatArrayTopic("Centers_y").getEntry(), 
        self.tagTable.getFloatArrayTopic("Positions_x").getEntry(), 
        self.tagTable.getFloatArrayTopic("Positions_y").getEntry(), 
        self.tagTable.getFloatArrayTopic("Positions_z").getEntry(),
        self.tagTable.getFloatArrayTopic("Roll").getEntry(),
        self.tagTable.getFloatArrayTopic("Pitch").getEntry(),
        self.tagTable.getFloatArrayTopic("Yaw").getEntry()
        for index in range(len(cX)):
            tag = _AprilTag(cX, cY, tX, tY, tZ, roll, pitch, yaw)
            tagList.append(tag)

        return tagList
        
            


    