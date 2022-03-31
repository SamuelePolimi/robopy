import time
import numpy as np
from enum import Enum

from darias_interface import Darias, DariasMode
from darias_space import Trajectory, JointGoal, CartGoal, Goal


class RecordMode(Enum):
    CartRecordMode = 0
    JointRecordMode = 1


class Record:
    """
    This class provides an utility for recording trajectories. It fixes the robot in teaching mode and then it records
    the trajectory.
    """

    def __init__(self, robot, record_mode=RecordMode.CartRecordMode, left=True, sampling_frequency=10):
        """
        Instantiating a recording object.

        :param robot: An instance of the robot we are interested in
        :type robot: Darias
        :param record_mode: In which mode we would like to record
        :type record_mode: RecordMode
        :param left: do we use the left arm?
        :type left: bool
        :param sampling_frequency: How many time do we sample in one second
        :type sampling_frequency: int

        """
        self.robot = robot
        self.left = left
        self.mode = record_mode
        self.trajectory = Trajectory([])
        self.sampling_frequency = sampling_frequency
        self.dt = 1. / sampling_frequency

    def record_fixed_duration(self, duration=10.):
        """
        Record a trajectory of fixed time-length.

        :param duration: Duration of the trajectory expressed in seconds.
        :type duration: float

        """

        self.robot.mode.set_mode(DariasMode.DariasKinestheticMode)

        for t in range(int(duration * self.sampling_frequency)):
            self.trajectory.goal_list.append(self._record_goal())
            time.sleep(self.dt)

    def _record_goal(self):
        if self.mode == RecordMode.JointRecordMode:
            if self.left:
                return JointGoal(self.robot.arms.left.position, duration=self.dt)
            else:
                return JointGoal(self.robot.arms.right.position, duration=self.dt)
        elif self.mode == RecordMode.CartRecordMode:
            if self.left:
                return CartGoal(self.robot.left_end_effector.position, duration=self.dt)
            else:
                return CartGoal(self.robot.right_end_effector.position, duration=self.dt)

    def conditional_record(self, callback_start, callback_end, max_duration=60.):
        """
        This is a conditional recording. the recording will start as soon as callback_start will return true.

        :param callback_start: Function which receives a goal as input, and produce a decision. When the decision is true, then the recording starts
        :param callback_end: Function which receives a goal as input, and produce a decision. When the decision is true, then the record will stop, and this method will return.
        :param max_duration: the overall process cannot exceed the max_duration, expressed in seconds.
        """
        duration = 0.
        self.robot.mode.set_mode(DariasMode.DariasKinestheticMode)

        while not callback_start(self._record_goal()) and duration < max_duration:
            time.sleep(self.dt)
            duration += self.dt

        goal = self._record_goal()
        while not callback_end(goal) and duration < max_duration:
            self.trajectory.goal_list.append(goal)
            goal = self._record_goal()
            time.sleep(self.dt)
            duration += self.dt

        if duration >= max_duration:
            print("Max duration reached")


class RecordingCallback:

    def __init__(self):
        pass

    def __call__(self, goal):
        """
        Make a decision based on the current goal.

        :param goal: current goal
        :type goal: Goal
        :return: true if we want to activate the event (e.g., ending the recording or starting it)
        :rtype: bool

        """


class WaitingToStart(RecordingCallback):
    """
    Simple starting callback for the conditional start of the recording.
    It activates the recording when the velocities of the joints exceed a certain threshold.
    """
    def __init__(self, threshold=0.01, verbose=True):
        """
        This creates a callback for conditional_recording. The recording will start only when the mean velocity of each
        joint or position is greater then the threshold.

        :param threshold: minimum mean velocity for the recording to start
        :type threshold: float
        :param verbose: prompt on the console the activation of the recording
        :type verbose: bool
        """
        RecordingCallback.__init__(self)
        self.threshold = threshold
        self.current_position = None
        self.verbose = verbose

    def __call__(self, goal):
        """
        If the velocity exceed a certain treshold, than the record start.

        :param goal:
        :type goal: Goal
        :return: Start the recording if true
        :rtype: bool
        """
        if self.current_position is None:
            self.current_position = goal.position
            return False
        elif np.abs(self.current_position - goal.position).mean()/goal.duration < self.threshold:
            self.current_position = goal.position
            return False
        if self.verbose:
            print("Recording started %s" % str(time.time()))
        return True


class WaitingToEnd(RecordingCallback):
    """
    Stop the recording when the velocities are lower than a specific threshold for a given resting-duration.
    """

    def __init__(self, threshold=0.01, duration=1., verbose=True):
        """
        Instantiate a "waiting to end" callback for stopping the recording.

        :param threshold: max velocity in order to consider the arm not moving
        :param duration: resting time in seconds
        """
        RecordingCallback.__init__(self)
        self.threshold = threshold
        self.current_position = None
        self.max_duration = duration
        self.duration = 0.
        self.verbose = verbose

    def __call__(self, goal):
        """
        Deactivates the recording when the velocities are below a certain threshold.

        :param goal:
        :type goal: Goal
        :return:
        :rtype: bool
        """
        if self.current_position is None:
            self.current_position = goal.position
            return False
        elif np.abs(self.current_position - goal.position).mean()/goal.duration < self.threshold:
            self.current_position = goal.position
            self.duration += goal.duration
            if self.duration > self.max_duration:
                if self.verbose:
                    print("Recording stopped %s" % str(time.time()))
                return True
        else:
            self.current_position = goal.position
            self.duration = 0.
        return False
