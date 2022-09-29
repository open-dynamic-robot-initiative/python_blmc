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

		self.update_gains(Kp, Ki, Kd)

	def update_data(self, mtr):
		self._mtr = mtr

	def update_gains(self, Kp, Ki, Kd):
		self._pid.SetKp(Kp)
		self._pid.SetKi(Ki)
		self._pid.SetKd(Kd)


	def run(self, refpos, verbose=True):
		error = refpos - self._mtr.position.value
		u = self._pid.GenOut(error, self._mtr.position.timestamp)
		self.iqref = u

		# clamp to allowed range
		self.iqref = min(self.iqref, self._maxval)
		self.iqref = max(self.iqref, -self._maxval)

		if verbose:
			print(("RefPos = {},\t\tPos = {:.4f}\t\tIqRef = {:.4f}\t\tdt [ms] = {:.2f}".format(
				refpos, self._mtr.position.value, self.iqref, self._pid.dt*1000.0)))


class VelocityController:

	def __init__(self, bus, Kp, Ki, Kd):
		self._bus = bus
		self._mtr = MotorData()
		self._pid = PID()
		self._maxval = 9.0
		self.iqref = 0

		self._pid.SetKp(Kp)
		self._pid.SetKi(Ki)
		self._pid.SetKd(Kd)

	def update_data(self, mtr):
		self._mtr = mtr

	def run(self, refspeed):
		#if self._status.mtr1_enabled and self._status.mtr1_ready: # FIXME
		error = refspeed - self._mtr.velocity.value
		u = self._pid.GenOut(error, self._mtr.velocity.timestamp)
		self.iqref = u

		# clamp to allowed range
		self.iqref = min(self.iqref, self._maxval)
		self.iqref = max(self.iqref, -self._maxval)

		print(("RefSpeed = {},\t\tSpeed = {:.4f}\t\tIqRef = {:.4f}\t\tdt [ms] = {:.0f}".format(
			refspeed, self._mtr.velocity.value, self.iqref, self._pid.dt*1000.0)))
