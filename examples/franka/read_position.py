from robopy.robots.franka import Franka
from robopy.observers import RobotObserver

if __name__ == "__main__":

    franka = Franka()
    observer = RobotObserver(franka)
    values = observer(*observer.get_possible_refs())

    print("Position of the Robot, %s" % values)