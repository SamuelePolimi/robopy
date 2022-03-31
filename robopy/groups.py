class Group:

    def __init__(self, group_name, refs):
        self.group_name = group_name
        self.refs = refs

    def __str__(self):
        return "%r: %s" % (self.group_name, str(self.refs))

    def __repr__(self):
        return self.__str__()


def group_union(group_name: str, group_1: Group, group_2: Group):
    return Group(group_name, group_1.refs + group_2.refs)
