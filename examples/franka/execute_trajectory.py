from robopy.robots.franka import Franka
from robopy.trajectory import LoadTrajectory
from robopy.movement_primitives import LearnTrajectory, ClassicSpace

if __name__ == "__main__":

    franka = Franka()

    trajectory_1 = LoadTrajectory("examples/franka/recorded_trajectory/recorded_trajectory.npy")

    learning_group = "arm"

    tr_init = trajectory_1.get_init_trajectory(10.)

    # Go to the correct position
    franka.go_to(tr_init, learning_group)

    print("Replay the trajectory")
    franka.go_to(trajectory_1, frequency=5, group_name=learning_group)
