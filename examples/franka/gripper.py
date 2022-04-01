from robopy.robots.franka import Franka
from robopy.trajectory import LoadTrajectory
from robopy.movement_primitives import LearnTrajectory, ClassicSpace

if __name__ == "__main__":

    franka = Franka()
    franka._gripper_interface.calibrate()
    franka._gripper_interface.close()
    franka._gripper_interface.stop_action()