"""
Small working example on how to access CAN bus.
"""
from __future__ import print_function
import os
import time
import can
import signal
import sys
from blmc.optoforce import *
from blmc.pid import PID
from blmc.motor_data import *
from blmc.conversion import *

BITRATE = 1e6

msg_enable_motor1 = can.Message(arbitration_id=0x000,
		data=[0, 0, 0, 1, 0, 0, 0, 2],
		extended_id=False)

msg_disable_motor1 = can.Message(arbitration_id=0x000,
		data=[0, 0, 0, 0, 0, 0, 0, 2],
		extended_id=False)

msg_ensable_system = can.Message(arbitration_id=0x000,
		data=[0, 0, 0, 1, 0, 0, 0, 1],
		extended_id=False)

msg_disable_system = can.Message(arbitration_id=0x000,
		data=[0, 0, 0, 0, 0, 0, 0, 1],
		extended_id=False)


def send_msg(bus, msg):
	"""Send a single message on the CAN bus."""
	try:
		bus.send(msg)
		#print("Message sent on {}".format(bus.channel_info))
	except can.CanError:
		print("Message NOT sent")



class VelocityController:

	def __init__(self, bus, Kp, Ki):
		self._bus = bus
		self._mtr1 = MotorData()
		self._status = Status()
		self._pid = PID()
		self._maxval = 9.0
		self._iqref = 0
		self._last_run = 0

		self._pid.SetKp(Kp)
		self._pid.SetKi(Ki)

	def run(self, refspeed):
		period = time.time() - self._last_run
		self._last_run = time.time()
		if self._status.mtr1_enabled and self._status.mtr1_ready:
			error = refspeed - self._mtr1.velocity
			u = self._pid.GenOut(error)
			self._iqref = u

			# clamp to allowed range
			self._iqref = min(self._iqref, self._maxval)
			self._iqref = max(self._iqref, -self._maxval)

			print("RefSpeed = {},\t\tSpeed = {:.4f}\t\tIqRef = {:.4f}\t\tdt [ms] = {:.0f}".format(
				refspeed, self._mtr1.velocity, self._iqref, period*1000.0))
			self.send_mtr1_current(self._iqref)

	def send_mtr1_current(self, iq_ref):
		data = [0, 0, 0, 0, 0, 0, 0, 0]
		data[0:4] = value_to_q_bytes(iq_ref)

		msg = can.Message(arbitration_id=0x005,
				data=data,
				extended_id=False)
		send_msg(self._bus, msg)


def handle_package(data):
	pkt = OptoForceDataPacket31()
	try:
		pkt.set_packet_bytes(data)
		return pkt.fz
	except ValueError, e:
		print("Error: {}".format(e))


if __name__ == "__main__":
	if len(sys.argv) != 4:
		print("Usage: {} goal_speed Kp Ki".format(sys.argv[0]))
		sys.exit(1)


	goal_speed = float(sys.argv[1])
	Kp = float(sys.argv[2])
	Ki = float(sys.argv[3])

	optodata = None
	opto_zero_offset = 0
	optofullscale = 1000.0
	optospeed = 0

	bus = can.interface.Bus(bitrate=BITRATE)

	print("Setup controller with Kp = {}, Ki = {}".format(Kp, Ki))
	print("Goal speed: {}".format(goal_speed))
	vctrl = VelocityController(bus, Kp, Ki)

	# setup sigint handler to disable motor on CTRL+C
	def sigint_handler(signal, frame):
			print('Stop motor and shut down.')
			vctrl.send_mtr1_current(0)
			send_msg(bus, msg_disable_motor1)
			sys.exit(0)
	signal.signal(signal.SIGINT, sigint_handler)

	print("Enable system...")
	send_msg(bus, msg_ensable_system)

	print("Enable motor...")
	vctrl.send_mtr1_current(0) # start with zero
	send_msg(bus, msg_enable_motor1)

	# wait a second for the initial messages to be handled
	time.sleep(1)

	# wait for messages and update data
	for msg in bus:
		arb_id = msg.arbitration_id
		if arb_id == 0x010:
			vctrl._status.set_status(msg.data)
		elif arb_id == 0x021:
			vctrl._mtr1.set_current_pos(msg.data)
		elif arb_id == 0x031:
			vctrl._mtr1.set_velocity(msg.data)

			# emergency break
			if vctrl._mtr1.velocity > goal_speed * 3:
				send_msg(bus, msg_disable_system)
				print("Too fast! EMERGENCY BREAK!")
				break

			# trigger vel ctrl everytime we get a new velocity
			vctrl.run(optospeed)

		elif arb_id == 0x100:
			#print([hex(x) for x in msg.data])
			
			# check for header 0xAA07080A
			if msg.data[0:4] == b"\xAA\x07\x08\x0A": #[0xAA, 0x07, 0x08, 0x0A]:
				optodata = msg.data
			elif optodata is not None:
				optodata += msg.data
				fz = handle_package(optodata)
				if (opto_zero_offset == 0):
					opto_zero_offset = fz
				optospeed = float(max(0, fz - opto_zero_offset)) / optofullscale * goal_speed
				# clear after data is handled
				optodata = None
			else:
				print("Unexpected package {}".format(
					[int(x) for x in msg.data]))
