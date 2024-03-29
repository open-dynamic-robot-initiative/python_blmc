# -------------------------------------------------------------------------------
# PID.py
# A simple implementation of a PID controller
# -------------------------------------------------------------------------------
# Example source code for the book "Real-World Instrumentation with Python"
# by J. M. Hughes, published by O'Reilly Media, December 2010,
# ISBN 978-0-596-80956-0.
# http://examples.oreilly.com/9780596809577/CH09/PID.py
# -------------------------------------------------------------------------------


class PID:
    """Simple PID control.

    This class implements a simplistic PID control algorithm. When first
    instantiated all the gain variables are set to zero, so calling
    the method GenOut will just return zero.
    """

    def __init__(self):
        # initialze gains
        self.Kp = 0
        self.Kd = 0
        self.Ki = 0
        self.dt = 0

        self.Initialize()

    def SetKp(self, invar):
        """Set proportional gain."""
        self.Kp = invar

    def SetKi(self, invar):
        """Set integral gain."""
        self.Ki = invar

    def SetKd(self, invar):
        """Set derivative gain."""
        self.Kd = invar

    def SetPrevErr(self, preverr):
        """Set previous error value."""
        self.prev_err = preverr

    def Initialize(self):
        # initialize delta t variables
        self.currtm = None
        self.prevtm = None

        self.prev_err = 0

        # term result variables
        self.Cp = 0
        self.Ci = 0
        self.Cd = 0

    def GenOut(self, error, time=None):
        """Performs a PID computation and returns a control value based on
        the elapsed time (dt) and the error signal from a summing junction
        (the error parameter).
        """
        if time is None:
            self.currtm = get_time()  # get t
        else:
            self.currtm = time

        # at first call, we don't have a valid prevtime and therefore cannot
        # comput a valid dt. Just set prev_err and prevtm and return with 0
        # (i.e. don't do something in the first step).
        if self.prevtm is None:
            self.prev_err = error
            self.prevtm = self.currtm
            return 0

        dt = self.currtm - self.prevtm  # get delta t
        de = error - self.prev_err  # get delta error

        self.Cp = self.Kp * error  # proportional term
        self.Ci += error * dt  # integral term

        self.Cd = 0
        if dt > 0:  # no div by zero
            self.Cd = de / dt  # derivative term

        self.prevtm = self.currtm  # save t for next pass
        self.prev_err = error  # save t-1 error
        self.dt = dt

        # sum the terms and return the result
        return self.Cp + (self.Ki * self.Ci) + (self.Kd * self.Cd)
