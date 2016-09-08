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
		self._maxval = 3.0
		self.iqref = 0

		self._pid.SetKp(Kp)
		self._pid.SetKi(Ki)
		self._pid.SetKd(Kd)

	def update_data(self, mtr):
		self._mtr = mtr

	def run(self, refpos):
		error = refpos - self._mtr.position
		u = self._pid.GenOut(error)
		self.iqref = u

		# clamp to allowed range
		self.iqref = min(self.iqref, self._maxval)
		self.iqref = max(self.iqref, -self._maxval)

		print("RefPos = {},\t\tPos = {:.4f}\t\tIqRef = {:.4f}\t\tdt [ms] = {:.2f}".format(
			refpos, self._mtr.position, self.iqref, self._pid.dt*1000.0))


