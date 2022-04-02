
from robopy.robots.franka import Franka, FrankaHome
from robopy.observers import RobotObserver
from robopy.recording import Recorder, WaitingToStart, WaitingToEnd
from robopy.trajectory import GoToTrajectory

if __name__ == "__main__":

    franka = Franka()
    observer = RobotObserver(franka)
    values = observer(*observer.get_possible_refs())

    recording = Recorder(observer, observer.get_possible_refs(), sampling_frequency=20)

    trajectory = GoToTrajectory(duration=10, **FrankaHome)
    franka.go_to(trajectory, "arm")

    input("Press Enter:")

    print("Start recording")
    recording.conditional_record(WaitingToStart(), WaitingToEnd(duration=0.5))
    print("End recording")

    print(recording.trajectory.refs, recording.trajectory.values)
    recording.trajectory.save("examples/franka/recorded_trajectory/recorded_trajectory.npy")
    print("file saved")

