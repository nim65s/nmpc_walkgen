class BaseTypeSupportFoot(object):

    def __init__(self, x=0, y=0, theta=0, foot='left'):
        self.x = x
        self.y = y
        self.q = theta
        self._foot = foot
        self._ds = 0
        self._stepNumber = 0
        self._timeLimit = 0

    def get_ds(self):
        return self._ds

    def set_ds(self, ds):
        self._ds = ds

    ds = property(get_ds, set_ds)

    def get_foot(self):
        return self._foot

    def set_foot(self, foot):
        self._foot = foot

    foot = property(get_foot, set_foot)

    def get_stepNumber(self):
        return self._stepNumber

    def set_stepNumber(self, stepNumber):
        self._stepNumber = stepNumber

    stepNumber = property(get_stepNumber, set_stepNumber) 

    def get_timeLimit(self):
        return self._timeLimit

    def set_timeLimit(self, timeLimit):
        self._timeLimit = timeLimit

    timeLimit = property(get_timeLimit, set_timeLimit)