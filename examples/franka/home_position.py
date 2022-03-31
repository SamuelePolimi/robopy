from robopy.robots.franka import Franka, FrankaHome
from robopy.trajectory import GoToTrajectory

if __name__ == "__main__":

    franka = Franka()
    trajectory = GoToTrajectory(duration=10, **FrankaHome)
    franka.go_to(trajectory, "arm")