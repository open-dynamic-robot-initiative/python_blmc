"""
Controllers
"""
import time
from .pid import PID
from .motor_data import MotorData

class PositionController:

	def __init__(self, Kp, Ki, Kd):
		self._mtr = MotorData()
		self._pid = PID()
		self._maxval = 9.0
		self.iqref = 0
		self._last_run = 0

		self._pid.SetKp(Kp)
		self._pid.SetKi(Ki)
		self._pid.SetKd(Kd)

	def update_data(self, mtr):
		self._mtr = mtr

	def run(self, refpos):
		period = time.time() - self._last_run
		self._last_run = time.time()

		error = refpos - self._mtr.position
		u = self._pid.GenOut(error)
		self.iqref = u

		# clamp to allowed range
		self.iqref = min(self.iqref, self._maxval)
		self.iqref = max(self.iqref, -self._maxval)

		print("RefPos = {},\t\tPos = {:.4f}\t\tIqRef = {:.4f}\t\tdt [ms] = {:.0f}".format(
			refpos, self._mtr.position, self.iqref, period*1000.0))


