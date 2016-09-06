"""
Helper classes to interpret data send by the board
"""
from .conversion import *


class ArbitrationIds:
	status = 0x010
	current = 0x020
	position = 0x030
	velocity = 0x040
	adc6 = 0x050
	optoforce_trans = 0x100
	optoforce_recv  = 0x101


class MessageHandler:
	
	def __init__(self):
		self._id_fnx_map = {}

	def set_id_handler(self, arbitration_id, func):
		self._id_fnx_map[arbitration_id] = func

	def handle_msg(self, arbitration_id, data):
		if arbitration_id in self._id_fnx_map:
			return self._id_fnx_map[arbitration_id](data)
		else:
			return None

class MotorDataStruct:
	current = 0
	position = 0
	velocity = 0

	def to_string(self):
		return "Iq: {:.3f},  Pos: {:.3f},  Speed: {:.3f}".format(
				self.current, self.position, self.velocity)


def MDL(data):
	return data[0:4]

def MDH(data):
	return data[4:8]

class MotorData:
	def __init__(self):
		self.mtr1 = MotorDataStruct()
		self.mtr2 = MotorDataStruct()
		self.status = Status()

	def set_status(self, data):
		self.status.set_status(data)

	def set_current(self, data):
		self.mtr1.current = q_bytes_to_value(MDL(data))
		self.mtr2.current = q_bytes_to_value(MDH(data))

	def set_position(self, data):
		self.mtr1.position = q_bytes_to_value(MDL(data))
		self.mtr2.position = q_bytes_to_value(MDH(data))

	def set_velocity(self, data):
		self.mtr1.velocity = q_bytes_to_value(MDL(data))
		self.mtr2.velocity = q_bytes_to_value(MDH(data))

	def to_string(self):
		return "{} | MTR1: {}\t | MTR2: {}".format(
				self.status.to_string(),
				self.mtr1.to_string(),
				self.mtr2.to_string())


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
		self.mtr2_ready = status_code & (1 << 4)
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


class AdcResult:
	a = 0
	b = 0

	def set_values(self, data):
		self.a = q_bytes_to_value(MDL(data))
		self.b = q_bytes_to_value(MDH(data))

	def to_string(self):
		return "ADC: {:.3f} / {:.3f}".format(self.a, self.b)
