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

msg_enable_motor2 = can.Message(arbitration_id=0x000,
		data=[0, 0, 0, 1, 0, 0, 0, 3],
		extended_id=False)

msg_disable_motor2 = can.Message(arbitration_id=0x000,
		data=[0, 0, 0, 0, 0, 0, 0, 3],
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

	def __init__(self, Kp, Ki):
		self._mtr1 = MotorData()
		self._status = Status()
		self._pid = PID()
		self._maxval = 9.0
		self.iqref = 0
		self._last_run = 0

		self._pid.SetKp(Kp)
		self._pid.SetKi(Ki)

	def run(self, refspeed):
		period = time.time() - self._last_run
		self._last_run = time.time()
		if self._status.mtr1_enabled and self._status.mtr1_ready:
			error = refspeed - self._mtr1.velocity
			u = self._pid.GenOut(error)
			self.iqref = u

			# clamp to allowed range
			self.iqref = min(self.iqref, self._maxval)
			self.iqref = max(self.iqref, -self._maxval)

			print("RefSpeed = {},\t\tSpeed = {:.4f}\t\tIqRef = {:.4f}\t\tdt [ms] = {:.0f}".format(
				refspeed, self._mtr1.velocity, self.iqref, period*1000.0))
			#self.send_mtr1_current(self.iqref)


def send_mtr_current(bus, mtr1_iqref, mtr2_iqref):
	data = [0, 0, 0, 0, 0, 0, 0, 0]
	data[0:4] = value_to_q_bytes(mtr1_iqref)
	data[4:8] = value_to_q_bytes(mtr2_iqref)

	msg = can.Message(arbitration_id=0x005,
			data=data,
			extended_id=False)
	send_msg(bus, msg)


def handle_optoforce_package(data):
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
	vctrl1 = VelocityController(Kp, Ki)
	vctrl2 = VelocityController(Kp, Ki)

	# setup sigint handler to disable motor on CTRL+C
	def sigint_handler(signal, frame):
			print('Stop motor and shut down.')
			send_mtr_current(bus, 0, 0)
			send_msg(bus, msg_disable_motor1)
			send_msg(bus, msg_disable_motor2)
			sys.exit(0)
	signal.signal(signal.SIGINT, sigint_handler)

	print("Enable system...")
	send_msg(bus, msg_ensable_system)

	print("Enable motor...")
	send_mtr_current(bus, 0, 0) # start with zero
	send_msg(bus, msg_enable_motor1)
	send_msg(bus, msg_enable_motor2)

	# wait a second for the initial messages to be handled
	time.sleep(1)

	# wait for messages and update data
	for msg in bus:
		arb_id = msg.arbitration_id
		if arb_id == 0x010:
			vctrl1._status.set_status(msg.data)
			vctrl2._status.set_status(msg.data)
			if not vctrl1._status.mtr1_ready:
				print("Waiting for motor 1...")
			if not vctrl2._status.mtr2_ready:
				print("Waiting for motor 2...")
		elif arb_id == 0x021:
			vctrl1._mtr1.set_current_pos(msg.data)
		elif arb_id == 0x022:
			vctrl2._mtr1.set_current_pos(msg.data)
		elif arb_id == 0x031:
			vctrl1._mtr1.set_velocity(msg.data)

			# emergency break
			if vctrl1._mtr1.velocity > goal_speed * 3:
				send_msg(bus, msg_disable_system)
				print("Motor 1 too fast! EMERGENCY BREAK!")
				break

			# trigger vel ctrl everytime we get a new velocity
			vctrl1.run(optospeed)
		elif arb_id == 0x032:
			vctrl2._mtr1.set_velocity(msg.data)

			# emergency break
			if vctrl2._mtr1.velocity > goal_speed * 3:
				send_msg(bus, msg_disable_system)
				print("Motor 2 too fast! EMERGENCY BREAK!")
				break

			# trigger vel ctrl everytime we get a new velocity
			vctrl2.run(optospeed)

			# 0x032 should arrive just after 0x031, so we use this place to
			# send the control outputs to the motor.
			send_mtr_current(bus, vctrl1.iqref, vctrl2.iqref)

		elif arb_id == 0x100:
			#print([hex(x) for x in msg.data])
			
			# check for header 0xAA07080A
			if msg.data[0:4] == b"\xAA\x07\x08\x0A": #[0xAA, 0x07, 0x08, 0x0A]:
				optodata = msg.data
			elif optodata is not None:
				optodata += msg.data
				fz = handle_optoforce_package(optodata)
				if (opto_zero_offset == 0):
					opto_zero_offset = fz
				optospeed = float(max(0, fz - opto_zero_offset)) / optofullscale * goal_speed
				# clear after data is handled
				optodata = None
			else:
				print("Unexpected package {}".format(
					[int(x) for x in msg.data]))
