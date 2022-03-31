#!/usr/bin/env python
"""
This module (currently empty) is needed to exposes frames relative to objects detected by the optitrack.

"""
import tf
import rospy


reference_frame = '/darias'


class Optitrack:

    def __init__(self):
        self.listener = tf.TransformListener()


class NotificationMissing(Exception):

    def __init__(self):
        Exception.__init__(self, "The frame has not been notified yet.")


class NotExistingFrame(Exception):

    def __init__(self, s):
        Exception.__init__(self, "The frame %s does not exist or has not been yet recognized." % s)


class Frame:

    def __init__(self, optitrack, frame_name):
        self.frame_name = frame_name
        self.optitrack = optitrack

    def get_cartesian_coordinates(self):
        try:
            return self.optitrack.listener.lookupTransform(reference_frame, '/%s' % self.frame_name, rospy.Time(0))
        except:
            raise NotExistingFrame(self.frame_name)



#TODO: just access to the published list of frames
# class TFFrames(RosListener):
#     """
#     Access to the objects
#     """
#
#     def __init__(self):
#         RosListener.__init__(self)
#         activate_listener()
#         rospy.Subscriber('/tf', TFMessage,
#                          self.get_callback())
#         self.frames = {}
#
#     def _callback(self, data):
#         """
#         Callback of TFTree
#
#         :param data:
#         :type data: TFMessage
#         :return:
#         """
#         for tf in data.transforms:
#             if tf.child_frame_id not in self.frames:
#                 self.frames[tf.child_frame_id] = Frame(tf)
#             else:
#                 self.frames[tf.child_frame_id].notify(tf)
#
#     def get_frame(self, name, reference=None, max_wait=1.):
#         """
#         This method provide the access to a frame, as soon as the information are available, and return a goal.
#
#         :param name: Name of the frame.
#         :param reference: Frame of reference. Compute the relative transformation between frames. Still to be implemented!!
#         :param max_wait: Maximum waiting time expressed in seconds. After this, we assume the frame does not exists and
#         and an error is arised.
#         :return:
#         :rtype: CartGoal
#         """
#         start_time = time.time()
#
#         while name not in self.frames and time.time() - start_time < max_wait:
#             pass
#
#         if name not in self.frames:
#             raise NotExistingFrame(name)
#
#         while time.time() - start_time < max_wait:
#             try:
#                 return self.frames[name].get_goal()
#             except NotificationMissing:
#                 pass
#
#         return self.frames[name].get_goal()
