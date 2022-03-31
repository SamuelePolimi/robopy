#!/usr/bin/env python
"""
This module contains the interface for Darias.
Darias is viewed as an object with a set of joints and two end-effectors.

The end-effector (left and right) expose their positions in 3D coordinates (with relative orientations).
The joints can be divided in two logical groups (left and right), and they expose the angle in radiants.
In future the joints will be divided in four groups (left-arm; left-hand; right-arm; right-hand).

Darias interfaces exposes also two methods, one for control and one for kinesthetic teaching.
"""
import rospy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from ias_robot_msgs.msg import GoToGoal, State, GoToAction
from ias_robot_msgs.srv import SettingsUpdate, SettingsUpdateRequest, KinestheticRequest, Kinesthetic, RobotInformation, RobotInformationRequest
import time

import actionlib
import numpy as np
from enum import Enum

from dariaspy.ros_listener import RosListener
from dariaspy.trajectory import NamedTrajectoryBase
from dariaspy.groups import Group
from dariaspy.movement_primitives import MovementPrimitive


class DariasMode(Enum):

    DariasCommandMode = 3
    DariasKinestheticMode = 4


class GeometricPoint(RosListener):

    def __init__(self):
        RosListener.__init__(self)
        self.translation = np.array([0., 0., 0.])
        self.rotation = np.array([0., 0., 0., 0.])

    def _callback(self, data):
        self.translation = np.array([
            data.transform.translation.x,
            data.transform.translation.y,
            data.transform.translation.z
        ])
        self.rotation = np.array([
            data.transform.rotation.w,
            data.transform.rotation.x,
            data.transform.rotation.y,
            data.transform.rotation.z
        ])

    @property
    def position(self):
        return np.concatenate([self.translation, self.rotation], axis=0)


class DariasModeController:

    def __init__(self):
        self.settings_service = rospy.ServiceProxy('/darias_control/darias/settings', SettingsUpdate)
        self.set_mode(DariasMode.DariasCommandMode)

    def set_mode(self, mode):
        """
        It sets a different mode.

        :param mode: DariasMode.<mode>
        :type mode: Enum

        :Example:

        >>> mode = DariasModeController()
        >>> mode.set_mode(DariasMode.DariasCommandMode)
        """
        settings_request = SettingsUpdateRequest()
        settings_request.mask = settings_request.MODE
        settings_request.settings.mode = mode
        try:
            self.settings_service(settings_request)
        except rospy.ServiceException as exc:
            print("Settings Service error: " + str(exc))
            return
        self.mode = mode


class Joint:
    """
    Defines a robotic joint.
    """

    def __init__(self, position, velocity):
        self.position = position
        self.velocity = velocity


class Hand:

    def __init__(self):
        self.fingers = [np.zeros(3) for _ in range(5)]

    def update(self, info):
        for i in range(5):
            self.fingers[i] = info[0+i*3:(i+1)*3]


class InfoExposer:

    def __init__(self, *info_esposers):
        """
        An info exposer object exposes the information about some quantities.
        It can also 
        :param info_esposers:
        """


class Arms(RosListener):
    """
    Defines the robotics' arms.
    We can access to the left arm or to the right one.
    """

    def __init__(self):
        RosListener.__init__(self)
        rospy.Subscriber('/darias_control/darias/joint_state', JointState,
                         self.get_callback())
        self.right = Joint(None, None)
        self.left = Joint(None, None)
        self.order = None
        self.info = {}
        self.update_fr = 0.
        self._last_time = time.time()

    def _callback(self, data):
        for i, ref in enumerate(data.name):
            self.info[ref] = data.position[i]

        if self.order is None:
            self.order = data.name[15:22] + data.name[:15] + data.name[37:44] + data.name[22:37]
        #TODO: old code!
        self.right.position = np.concatenate([data.position[15:22], data.position[:15]], axis=0)
        self.left.position = np.concatenate([data.position[37:44], data.position[22:37]], axis=0)
        self.right.velocity = np.concatenate([data.velocity[15:22], data.velocity[:15]], axis=0)
        self.left.velocity = np.concatenate([data.velocity[37:44], data.velocity[22:37]], axis=0)


class EndEffector(GeometricPoint):

    def __init__(self, left=True):
        GeometricPoint.__init__(self)
        rospy.Subscriber('/darias_control/darias/ENDEFF_%s_ARM_pos' % ('LEFT' if left else 'RIGHT'), TransformStamped,
                         self.get_callback())


class Darias:

    def __init__(self, local=True):
        """
        Instantiates an interface for darias.
        """

        ip = "127.0.0.1" if local else "192.168.1.102"

        self.client = rc.Client(ip, 2013, "darias", max_payload_size=5*1012, max_buffer_size=5*1024*1024)
        #########################################
        # State
        #########################################
        self.left_end_effector = EndEffector(True)
        self.right_end_effector = EndEffector(False)
        self.arms = Arms()
        self.mode = DariasModeController()

        self.frequency = 1000.

        #########################################
        # Action
        #########################################
        self._handle_goto_service = actionlib.SimpleActionClient('/darias_control/darias/goto', GoToAction)
        self._handle_goto_service.wait_for_server()

        self.groups = {}
        self._building_groups()


        while not(self.arms.ready and self.left_end_effector.ready and self.right_end_effector.ready):
            pass

    def go_to(self, trajectory, group_name, goal_weight=1.0, wait=True):
        """
        It performs a desired trajectory.

        :param trajectory: trajectory to follow
        :type trajectory: NamedTrajectoryBase
        :param group_name: Name of the group
        :param goal_weight: How much the velocity in the via-points:
        :type goal_weight: float
        :type group_name: str
        """
        self.mode.set_mode(DariasMode.DariasCommandMode)

        group = group_name
        if group_name == 'ENDEFF_LEFT_ARM':
            group = "LEFT_ARM"
        elif group_name == 'ENDEFF_RIGHT_ARM':
            group = "RIGHT_ARM"

        # Construct the Command Message:
        joint_goal = GoToGoal()
        if group_name in ['ENDEFF_LEFT_ARM', 'ENDEFF_RIGHT_ARM']:
            joint_goal.type = joint_goal.CART
        else:
            joint_goal.type = joint_goal.JOINT

        joint_goal.group = group

        joint_goal.weight = goal_weight
        joint_goal.priority = joint_goal.APPEND

        for goal, d in trajectory:
            joint_state = State()
            joint_state.duration = d
            joint_state.destination = [goal[k] for k in self.groups[group_name].refs]
            joint_goal.states.append(joint_state)

        self._handle_goto_service.send_goal(joint_goal)
        if wait:
            self._handle_goto_service.wait_for_result()

    def kinesthetic(self, group):
        """
        This method provide the kinerthetic teaching (i.e., gravity compensation mode).
        In order to stop this service it is sufficient to call mode.set_mode(DariasCommandMode)

        :param left: left or right arm?
        :type left: bool
        """

        self.mode.set_mode(DariasMode.DariasKinestheticMode)

        rospy.wait_for_service('/darias_control/darias/kinesthetic')
        kin_service = rospy.ServiceProxy('darias_control/darias/kinesthetic', Kinesthetic)
        start_msg = KinestheticRequest()
        start_msg.mirrored = False
        start_msg.teached_group = self.groups[group].group_name

        try:
            kin_service(start_msg)
        except rospy.ServiceException as exc:
            print("Kinesthetic Service error: " + str(exc))
            return

    def goto_mp(self, mp, duration=10., wait=True):
        """

        :param mp: The MP already defined between 0. and 1.
        :type mp: MovementPrimitive
        :param duration: The duration of the movement
        :type duration: float
        :param wait: True if we want to wait the end of the movement.
        :type wait: bool
        :return: if wait==False then it returns a function which, when called, will wait for the completion of the movement
        """
        centers = mp.movement_space.centers
        scales = mp.movement_space.bandwidths

        mp_type = rc.Client.teaching_joint_space_promp
        group = mp.movement_space.group.group_name
        if mp.movement_space.group.group_name == 'ENDEFF_LEFT_ARM':
            mp_type = rc.Client.teaching_task_space_promp
            group = "LEFT_ARM"
        elif mp.movement_space.group.group_name == 'ENDEFF_RIGHT_ARM':
            mp_type = rc.Client.teaching_task_space_promp
            group = "RIGHT_ARM"
        pmp = self.client.get(mp_type, group)
        rbf_args = np.zeros(2 * len(mp.movement_space.centers))
        rbf_args[0::2] = centers
        rbf_args[1::2] = scales

        #TODO: what is that?
        wi_args = np.array([0.5])

        fr_required = self.frequency * duration
        dz = 1. / fr_required

        w = np.array([mp.params[ref] for ref in mp.movement_space.group.refs])

        self.client.mode = rc.Client.mode_teaching
        pmp.setup(pmp.rbf, rbf_args, w, pmp.linear, wi_args, 0., 1., dz)

        pmp.start()

        if not wait:
            return pmp.stop
        pmp.stop()

    def _building_groups(self):

        rospy.wait_for_service('/darias_control/darias/information')
        service = rospy.ServiceProxy('/darias_control/darias/information', RobotInformation)
        req = RobotInformationRequest()
        try:
            answer = service(req)
            for group in answer.joint_space_control:
                self.groups[group.name] = Group(group.name, group.joints)
            self.groups["ENDEFF_LEFT_ARM"] = Group("ENDEFF_LEFT_ARM",
                                                   ["L_TX", "L_TY", "L_TZ", "L_RX", "L_RY", "L_RZ", "L_RW"])
            self.groups["ENDEFF_RIGHT_ARM"] = Group("ENDEFF_RIGHT_ARM",
                                                    ["R_TX", "R_TY", "R_TZ", "R_RX", "R_RY", "R_RZ", "R_RW"])

        except rospy.ServiceException as exc:
            print("Information Service error: " + str(exc))
            return

