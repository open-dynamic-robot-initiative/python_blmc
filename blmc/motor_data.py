"""
Helper classes to interpret data send by the board
"""
from .conversion import *

class MotorData:
	current = 0
	position = 0
	velocity = 0

	def set_current_pos(self, data):
		self.current = q_bytes_to_value(data[0:4])
		self.position = q_bytes_to_value(data[4:8])

	def set_velocity(self, data):
		self.velocity = q_bytes_to_value(data)

	def to_string(self):
		return "Iq: {:.3f},  Pos: {:.3f},  Speed: {:.3f}".format(
				self.current, self.position, self.velocity)


class Status:
	system_enabled = 0
	mtr1_enabled = 0
	mtr1_ready = 0
	mtr2_enabled = 0
	mtr2_ready = 0
	mtr1_overheat = 0
	mtr2_overheat = 0
	system_error = 0

	def set_status(self, data):
		status_code = data[0]
		self.system_enabled = status_code & 1
		self.mtr1_enabled = status_code & (1 << 1)
		self.mtr1_ready = status_code & (1 << 2)
		self.mtr2_enabled = status_code & (1 << 3)
		self.mtr2_enabled = status_code & (1 << 4)
		self.mtr1_overheat = status_code & (1 << 5)
		self.mtr2_overheat = status_code & (1 << 6)
		self.system_error = status_code & (1 << 7)

	def to_string(self):
		str_sys = "SYS: "
		str_sys += "E" if self.system_enabled else "D"
		str_sys += "!!!" if self.system_error else ""

		str_mtr1 = "MTR1: "
		str_mtr1 += "E" if self.mtr1_enabled else "D"
		str_mtr1 += "R" if self.mtr1_ready else "A"

		str_mtr2 = "MTR2: "
		str_mtr2 += "E" if self.mtr2_enabled else "D"
		str_mtr2 += "R" if self.mtr2_ready else "A"

		return "{} | {} | {}".format(str_sys, str_mtr1, str_mtr2)
