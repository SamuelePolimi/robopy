from robopy.robots.franka import Franka
from robopy.trajectory import LoadTrajectory
from robopy.movement_primitives import LearnTrajectory, ClassicSpace

if __name__ == "__main__":

    franka = Franka()

    trajectory_1 = LoadTrajectory("examples/franka/recorded_trajectory/recorded_trajectory.npy")

    learning_group = "arm"

    ms = ClassicSpace(franka.groups[learning_group], n_features=30)

    # Learn te movement in this space
    mp1 = LearnTrajectory(ms, trajectory_1)
    print("Go to the initial point of the movement primitive")
    tr_init = mp1.get_init_trajectory(10.)

    # Go to the correct position
    franka.go_to(tr_init, learning_group)

    print("Play the movement primitive")
    # Go to with movement primitive

    franka.goto_mp(mp1, frequency=5, group_name=learning_group)
