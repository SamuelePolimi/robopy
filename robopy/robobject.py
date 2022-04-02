import numpy as np
from robopy.groups import Group, group_union

class RobObject:

    def __init__(self, object_name):
        self._object_name = object_name
        self.refs = []
        self.groups = {}
        self._group_loaded = False
        self._limits = {}


    def get_name(self) -> str:
        return self._object_name

    def set_limit(self, ref, min_value, max_value):
        self._limits[ref] = (min_value, max_value)

    def set_groups(self, *groups: Group):
        self.groups = {}
        g = Group(self._object_name, [])
        for g1 in groups:
            g = group_union(self._object_name, g, g1)
            self.groups[g1.group_name] = g1
        self.groups[self._object_name] = g
        self.refs = g.refs
        for ref in self.refs:
            self._limits[ref] = (-np.inf, np.inf)

    def get_groups(self) -> Group:
        return self._groups

    def read(self, *refs):
        raise NotImplemented()

    def __call__(self, *refs):
        observation = self.read(*refs)
        for k in observation.keys():
            val = observation[k]
            limit = self._limits[k]
            if limit[0] > val or limit[1] < val:
                raise Warning("%s=%f not in (%f, %f)" % (k, val, limit[0], limit[1]))
            observation[k] = np.clip(val, limit[0], limit[1])
        return observation
