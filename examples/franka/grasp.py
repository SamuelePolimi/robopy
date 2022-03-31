from robopy.robots.franka import Franka
import time

if __name__ == "__main__":

    franka = Franka()
    franka._gripper_interface.open()
    franka._gripper_interface.stop_action()
    franka._gripper_interface.grasp(0.1, 0.1)

    time.sleep(3)

    franka._gripper_interface.stop_action()