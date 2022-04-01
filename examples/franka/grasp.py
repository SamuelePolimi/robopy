from robopy.robots.franka import Franka
import time

if __name__ == "__main__":

    franka = Franka()
    franka._gripper_interface.open()
    franka._gripper_interface.stop_action()
    franka._gripper_interface.move_joints(0.05)

    time.sleep(10)

    franka._gripper_interface.stop_action()