class linePoint(object):
    """Represents a line segment by its 2 endpoints"""
    def __init__(self, array):
        self.p0 = (array[0][0], array[0][1])
        self.p1 = (array[0][2], array[0][3])

    def __str__(self):
        return "{}, {}".format(self.p0, self.p1)
