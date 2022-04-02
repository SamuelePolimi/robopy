from robopy.observers import MissingRefException
import numpy as np


class InconsistentException(Exception):

    def __init__(self):
        Exception.__init__(self, "The notification is inconsistent.")


class NamedPoint:
    """
    This represent a combination of refs and correspondent values in a given instant.
    """

    def __init__(self, *refs):
        self.refs = refs
        self.values = np.zeros(len(self.refs))

    def set(self, **values):
        if len(values.values()) != self.refs:
            raise InconsistentException()
        for k in values:
            if not k in self.refs:
                raise MissingRefException(k)
            indx = self.refs.index(k)
            self.values[indx] = values[k]


class NamedTrajectoryBase:
    """
    A trajectory is a ordered sequence of NamedPoints with the duration of the transition between one point and the following.
    """

    def __init__(self, refs, durations, values):
        """

        :param refs: list of M references
        :param durations: list of N durations
        :param values: NxM matrix of N points , each row contains the value for each 0:M-1 reference
        """
        self.refs = refs
        self.duration = durations
        self.values = values

    def __iter__(self):
        for values, d in zip(self.values, self.duration):
            yield {r: v for r, v in zip(self.refs, values.ravel())}, d

    def save(self, filename):
        """
        Save the trajectory on file
        :param filename: Name of the file
        :type filename: str
        :rtype: None
        """
        ret = {
           'refs': list(self.refs),
            'duration': self.duration,
            'values': self.values
        }
        np.save(filename, ret)

    def get_sub_trajectory(self, *refs):
        """
        Extract a trajectory with a subset of references.
        :param refs: references
        :type refs: str
        :return: a sub NamedTrajectory
        :rtype: NamedTrajectoryBase
        """
        ret = self._get_movement(*refs)
        return NamedTrajectoryBase(refs, self.duration, np.array(ret).T)

    def get_dict_values(self):
        return {ref: value for ref, value in zip(self.refs, self.values.T)}

    def _get_movement(self, *refs):

        ret = []
        for ref in refs:
            indx = self.refs.index(ref)
            ret.append(self.values[:, indx])
        return ret

    def get_init_trajectory(self, duration: float):
        ret = {}
        for k in self.refs:
            indx = self.refs.index(k)
            ret[k] = self.values[0][indx]
        return GoToTrajectory(duration, **ret)

    def plot(self, axis):
        for i, k in enumerate(self.refs):
            axis[i].plot(np.cumsum(self.duration), self.values[:, i])
            axis[i].set_title(k)


class GoToTrajectory(NamedTrajectoryBase):
    """
    A trajectory composed of only one point to e reached.
    """

    def __init__(self, duration=10., **values):
        NamedTrajectoryBase.__init__(self, values.keys(),
                                     np.array([duration]), np.array([[values[ref] for ref in values.keys()]]))


def LoadTrajectory(filename):
    """
    Load a named trajectory from file.
    :param filename: file
    :type filename: str
    :return: the named-trajectory extracted from the file
    :rtype: NamedTrajectoryBase
    """
    obj = np.load(filename, allow_pickle=True)
    return NamedTrajectoryBase(obj.item()['refs'], obj.item()['duration'], obj.item()['values'])


class NamedTrajectory(NamedTrajectoryBase):
    """
    Empty named trajectory. It is possible to notify new values and record the trajectory.
    """

    def __init__(self, *refs):
        NamedTrajectoryBase.__init__(self, refs, np.zeros(0), np.zeros((0, len(refs))))

    def notify(self, duration=0.1, **values):
        """
        Add one point to the trajectory.
        :param duration: number of seconds from the previous point
        :param values: value for each reference
        :return: None
        :rtype: None
        """
        if len(values.values()) != len(self.refs):
            raise InconsistentException()
        app_values = np.zeros(len(self.refs))
        for k in values:
            if k not in self.refs:
                raise MissingRefException(k)
            indx = self.refs.index(k)
            app_values[indx] = values[k]
        concat = app_values.reshape(1, -1)
        self.values = np.concatenate([self.values, concat], axis=0)
        self.duration = np.concatenate([self.duration, np.array([duration])], axis=0)
