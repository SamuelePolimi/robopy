from robopy.observers import Observer, MissingRefException
from optitrack import Frame

_v2_reader = {
    'X': 0,
    'Y': 1,
    'Z': 2,
    'W': 3
}

_v1_reader = {
    'T': 0,
    'R': 1
}

_refs = (
    "%s_TX",
    "%s_TY",
    "%s_TZ",
    "%s_RX",
    "%s_RY",
    "%s_RZ",
    "%s_RW"
)


class OptitrackObserver(Observer):
    """
        Observe only the left and the right end-effectors in cartesian space.
        """

    def __init__(self, frame_name):
        Observer.__init__(self)
        self.frame = Frame(frame_name)

    def __call__(self, *ref_list):
        ret = {}
        observation = self.frame.get_cartesian_coordinates()
        for ref in ref_list:
            i_1, i_2 = self._getIndexObs(ref)
            ret[ref] = observation[i_1][i_2]
        return ret

    def _getIndexObs(self, ref_name):

        vals = ref_name.split("_")
        v1, v2 = vals[-1][0], vals[-1][1]
        if "_".join(vals[:-1]) != self.frame.frame_name:
            raise MissingRefException(ref_name)

        try:
            return _v1_reader[v1], _v2_reader[v2]
        except:
            raise MissingRefException(ref_name)

    def get_possible_refs(self):
        return [ref % self.frame.frame_name for ref in _refs]
