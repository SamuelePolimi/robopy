RoboPy
========

The goal of this library is to provide an easy, object-oriented interaction with robots and generic hardware for robotic experiments.
The main operation consist in sending commands to the robot (like goto position, or executing trajectory), and to also observe current robot position
(both in task and joint space), and to record trajectories (of robotic joints, task configuration, object state and positions).

The robotic interfaces are contained in `robopy.robots`. Currently there is an interface for `robopy.robots.darias` (a Kuka light-weight arm manipulator in the IAS group), and `robopy.robots.franka` (a Emika Franka robotic manipulator from the RLAI lab in University of Alberta).
Both the implementations of the interfaces are "context dependent", meaning that they rely on how the particular substrate of software (ROS modules, controllers, network configuration).
However, if one inspects the actual implementations provided for `franka` and `darias`, can see that they are simple, and therefore, one can easily customize this library for new robots (I plan myself to add a few more).

The library offers also a few interfaces wfor some hardware, in particular: a OptiTrack motion capture system (`hardware.optitrack`) and a Dymo Scale (`hardware.scale`).

General Concepts
-

*Robots and Hardware*

As said, the library offers some interface to interact with objects, for example the franka robot `from robopy.robots.franka import Franka`, or the scale `from robopy.hardware.scale import DymoScale`.
Such objects expose methods to interact with them, e.g., 

*Groups and Observers*

Each object exposes some measurable quantities, for example the robots has a joint configurations and the scale measures some weight.
An `Observer` tells you which measurable quantities are exposed by an object via `Observer.get_refs()`. And some quantities can be read using the observer
`Observer(**refs)`. Usually, quantities are observed in group (i.e., the joints, the task space, ...) and, therefore, can grouped together using `from robopy.groups import Group`.

*Recording and Trajectories*

One can record the measures exposed by an observed using `from robopy.recording import Recorder`. `Recorder` saves measurements with a desired frequency. The output of a `Recorder` is, hence, a `Trajectory` (sequence of observations).
`Trajectory` can be saved on disk, loaded from disk, and are usually the input of `goto` methods for the robot (i.e., the robot can play a recorded trajectory).
This generalization allows more sophisticated things. For example one can record the trajectory of an object in the scene, and the robot can play this trajectory with the end-effector.

Let's now give some examples of how to use the library!


Set the Robot in Home Position
-

```python
from robopy.robots.franka import Franka, FrankaHome
from robopy.trajectory import GoToTrajectory

if __name__ == "__main__":

    franka = Franka()
    trajectory = GoToTrajectory(duration=10, **FrankaHome)
    franka.go_to(trajectory, "arm")
```

Read the Currennt Joint Configuration:
-

```python
from robopy.robots.franka import Franka
from robopy.observers import RobotObserver

if __name__ == "__main__":

    franka = Franka()
    observer = RobotObserver(franka)
    values = observer(*observer.get_possible_refs())
    print("Position of the Robot, %s" % values)
```

Record and Save a Robotic Motion
-

```python
from robopy.robots.franka import Franka, FrankaHome
from robopy.observers import RobotObserver
from robopy.recording import Recorder
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
    recording.record_fixed_duration(10.)
    print("End recording")

    print(recording.trajectory.refs, recording.trajectory.values)
    recording.trajectory.save("examples/franka/recorded_trajectory/recorded_trajectory.npy")
    print("file saved")
```

Execute a Recorded Robotic Motion
-

```python
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
```

Read the DymoScale
--

```python
from robopy.hardware.scale import DymoScale, ScaleObserver

if __name__ == "__main__":

    scale = DymoScale()
    scale_observer = ScaleObserver(scale)
    print("The current weight measured is: %dg" % scale_observer("DYMO_WEIGHT")["DYMO_WEIGHT"])
```


References to Darias
--------------------

How to install the needed software (Ros, IASRos, Robcom) [Guide by Koert](https://git.ias.informatik.tu-darmstadt.de/ias_ros/ias_ros_core).
How to start Darias [Guide by Stark and Koert](https://git.ias.informatik.tu-darmstadt.de/ausy/wiki/blob/master/tutorial_darias_right_arm/Using_DARIAS_Right_Arm2.pdf).
How to mount the DLR hand and use it [DLR Hands Guide](dariashand.pdf).

Start ros with:
````bash
roslaunch robcom_launch darias.launch
````

Start sl with:
```bash
cd ~/sl/build/darias/
./xdarias
```

References to Franka
----

_TODO_


Goals
-----

- [x] Initialize the library
- [x] Perceive cartisian and joint current positions
- [x] GoTo with joint coordinates
- [x] GoTo with 3D coordinates
- [x] Wait GoTo for finish
- [x] Definition of Trajectory
- [x] Robot mode
- [x] Record Trajectory 
- [x] Points from optitrack
- [x] Transormation relative to a reference-frame
- [x] Create `observers`. Will be useful either for distinguish arm and hand, an also for generalizing recording.
- [x] Record extended/generalized to optitrack objects
- [x] Hand control
- [x] ProMPs execution
- [x] Renaming in `robopy`
- [x] Integrate Dynamo Scale
- [x] Move to Python 3
- [x] General interface with other robots
- [ ] Better guide, finer documentation
- [ ] Exposition of shell commands
- [ ] Gui interface for status and commands
- [ ] Integration with cameras
- [ ] Integration with OpenCV
- [ ] More common interfaces for Robots, and for Hardware
- [ ] Generalize more Observers
- [ ] Remove Movement Primitives and use them only from ROMI
- [ ] Visualization class (at least for trajectories)
- [ ] Would be nice an integrazion with Simulators (CoppeliaSim, MuJoCo)