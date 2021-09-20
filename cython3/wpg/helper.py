class BaseTypeSupportFoot(object):

    def __init__(self, x=0, y=0, theta=0, foot="left"):
        self.x = x
        self.y = y
        self.q = theta
        self.foot = foot
        self.ds = 0
        self.stepNumber = 0
        self.timeLimit = 0

