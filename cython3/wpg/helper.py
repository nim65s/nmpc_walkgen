class BaseTypeSupportFoot(object):

    def __init__(self, x=0, y=0, theta=0, foot="left"):
        self.x = x
        self.y = y
        self.q = theta
        self.foot = foot
        self._ds = 0
        self.stepNumber = 0
        self.timeLimit = 0

    def get_ds(self):
        return self._ds

    def set_ds(self, ds):
        self._ds = ds

    ds = property(get_ds, set_ds)
