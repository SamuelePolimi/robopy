RoboPy
========

> **WARNING**: THIS README IS OUTDATED! WORKING ON PROGRESS!
>
> **WARNING**: The new updates requires **robcom interface for python** (recompile robcom allowing in the configuration python). For further informations ask `samuele.tosatto@tu-darmstadt.de`.

> **WARNING**: From now on it is necessary to import **activate listener** from dariaspy.ros_listener and execute it before anything else.

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


The library already exposes some nice features, like the possibility to starting the trajectory conditioning on a specific
event (such as velocity exceeding a threshold), but it will be extended in the near future with more specific funtionalities.

A long-term goal would ideally be to have a simpler and common interface to all our robots :).

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
- [x] Renaming in `robopy` or `iaspy`, ...?
- [x] Integrate Dynamo Scale
- [ ] Move to Robcom2 once it offers promps
- [x] Move to Python 3
- [x] General interface with other robots
- [ ] Better guide, finer documentation
- [ ] Exposition of shell commands
- [ ] Gui interface for status and commands