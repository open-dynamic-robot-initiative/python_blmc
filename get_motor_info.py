"""
Small working example on how to access CAN bus.
"""
from __future__ import print_function
import os
import struct
import can


def send_one(bus):
	"""Send a single message on the CAN bus."""
	# enable motor 1
    msg = can.Message(arbitration_id=0x000,
                      data=[0, 0, 0, 1, 0, 0, 0, 2],
                      extended_id=False)
    try:
        bus.send(msg)
        print("Message sent on {}".format(bus.channel_info))
    except can.CanError:
        print("Message NOT sent")


def q_bytes_to_value(data):
	"""Convert bytes recevied via CAN to float value."""

	assert len(data) == 4
	# convert bytes to signed int (note: bytes in data are in reversed order)
	qval = struct.unpack("<i", data[::-1])[0]
	# convert q-value to human-readable float
	fval = float(qval) / 2.0**24
	return fval


class MotorData:
	current = 0
	position = 0
	velocity = 0

	def set_current_pos(self, data):
		self.current = q_bytes_to_value(data[0:4])
		self.position = q_bytes_to_value(data[4:8])

	def set_velcity(self, data):
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


if __name__ == "__main__":
	bus = can.interface.Bus(bitrate=250000)
	mtr1 = MotorData()
	mtr2 = MotorData()
	status = Status()

	# wait for messages and update data
	for msg in bus:
		arb_id = msg.arbitration_id
		if arb_id == 0x010:
			status.set_status(msg.data)
		elif arb_id == 0x021:
			mtr1.set_current_pos(msg.data)
		elif arb_id == 0x022:
			mtr2.set_current_pos(msg.data)
		elif arb_id == 0x031:
			mtr1.set_velcity(msg.data)
		elif arb_id == 0x032:
			mtr2.set_velcity(msg.data)

		#os.system('cls' if os.name == 'nt' else 'clear')
		print("\r{}   \tMTR1: {}   \tMTR2: {}".format(
			status.to_string(), mtr1.to_string(), mtr2.to_string()),)
