from robopy.observers import Observer, MissingRefException


class ScaleObserver(Observer):
    """
    Observe the weight measured from DymoScale
    """

    def __init__(self, scale):
        """
        :param scale: The interface of the scale
        """
        Observer.__init__(self)
        self.scale = scale

    def __call__(self, *ref_list):
        ret = {}
        for ref in ref_list:
            if ref == "%s_WEIGHT" % self.scale.get_name():
                ret[ref] = self.scale.get_weight()
            elif ref == "%s_TARE" % self.scale.get_name():
                ret[ref] = self.scale.get_tare()
            else:
                raise MissingRefException(ref)
        return ret

    def get_possible_refs(self):
        return ["%s_WEIGHT" % self.scale.get_name(),
                "%s_TARE" % self.scale.get_name()]