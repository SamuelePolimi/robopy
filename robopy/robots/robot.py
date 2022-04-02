from robopy.robobject import RobObject
from robopy.trajectory import NamedTrajectory

class Robot(RobObject):

    def __init__(self, robot_name):
        RobObject.__init__(self, robot_name)

    def goto(self, trajectory: NamedTrajectory, group_name):
        raise NotImplemented()