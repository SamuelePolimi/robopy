from robopy.robots.franka import Franka
from robopy.trajectory import LoadTrajectory, ExponentialSmoothing
import matplotlib.pyplot as plt


if __name__ == "__main__":

    franka = Franka()

    trajectory = LoadTrajectory("examples/franka/recorded_trajectory/recorded_trajectory.npy")

    fig, axis = plt.subplots(len(franka.groups["arm_gripper"].refs))
    trajectory.plot(axis)

    smoothing = ExponentialSmoothing()
    smooth_trajectory = smoothing.smooth(trajectory)

    smooth_trajectory.plot(axis)
    plt.savefig("trajectories.jpg")
