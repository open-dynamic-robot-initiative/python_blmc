"""
Small working example on how to access CAN bus.
"""
from __future__ import print_function
import os
import time
import can
import signal
import sys
import traceback
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



class PositionController:

	def __init__(self, Kp, Ki, Kd):
		self._mtr = MotorData()
		self._status = Status()
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
	if len(sys.argv) != 8:
		print("Usage: {} goal_pos Kp1 Ki1 Kd1 Kp2 Ki2 Kd2".format(sys.argv[0]))
		sys.exit(1)


	goal_pos = float(sys.argv[1])
	Kp1 = float(sys.argv[2])
	Ki1 = float(sys.argv[3])
	Kd1 = float(sys.argv[4])
	Kp2 = float(sys.argv[5])
	Ki2 = float(sys.argv[6])
	Kd2 = float(sys.argv[7])

	of_packet_receiver = OptoForcePacketReceiver()
	optofullscale = 1000.0
	optopos = 0

	mtr_data = MotorData()
	bus = can.interface.Bus(bitrate=BITRATE)

	# setup sigint handler to disable motor on CTRL+C
	def sigint_handler(signal, frame):
			print('Stop motor and shut down.')
			send_mtr_current(bus, 0, 0)
			send_msg(bus, msg_disable_motor1)
			send_msg(bus, msg_disable_motor2)
			sys.exit(0)
	signal.signal(signal.SIGINT, sigint_handler)


	print("Setup controller 1 with Kp = {}, Ki = {}, Kd = {}".format(
		Kp1, Ki1, Kd1))
	print("Setup controller 2 with Kp = {}, Ki = {}, Kd = {}".format(
		Kp2, Ki2, Kd2))
	print("Goal position: {}".format(goal_pos))
	vctrl1 = PositionController(Kp1, Ki1, Kd1)
	vctrl2 = PositionController(Kp2, Ki2, Kd2)

	print("Initialize OptoForce...")
	ofconf = OptoForceConfig()
	ofconf.zero()
	#ofconf.set_sample_frequency(
	#		OptoForceConfig.SampleFreq.Hz_1000)
	ofconf.send_via_can(bus, ArbitrationIds.optoforce_recv)

	print("Enable system...")
	send_msg(bus, msg_ensable_system)

	print("Enable motors...")
	send_mtr_current(bus, 0, 0) # start with zero
	send_msg(bus, msg_enable_motor1)
	send_msg(bus, msg_enable_motor2)

	# wait a moment for the initial messages to be handled
	time.sleep(0.2)


	def on_position_msg(data):
		mtr_data.set_position(data)

		if mtr_data.status.mtr1_ready:
			vctrl1.update_data(mtr_data.mtr1)
			vctrl1.run(optopos)

		if mtr_data.status.mtr2_ready:
			vctrl2.update_data(mtr_data.mtr2)
			vctrl2.run(optopos)

		#print(mtr_data.to_string())
		send_mtr_current(bus, vctrl1.iqref, vctrl2.iqref)

	def on_optoforce_msg(data):
		global optopos

		ofpkt = of_packet_receiver.receive_frame(data)
		if ofpkt is not None:
			optopos = float(max(0, ofpkt.fz)) / optofullscale * goal_pos

	msg_handler = MessageHandler()
	msg_handler.set_id_handler(ArbitrationIds.status, mtr_data.set_status)
	msg_handler.set_id_handler(ArbitrationIds.current, mtr_data.set_current)
	msg_handler.set_id_handler(ArbitrationIds.position, on_position_msg)
	msg_handler.set_id_handler(ArbitrationIds.velocity, mtr_data.set_velocity)
	#msg_handler.set_id_handler(0x050, mtr_data.set_status)
	msg_handler.set_id_handler(ArbitrationIds.optoforce_trans, on_optoforce_msg)


	# wait for messages and update data
	for msg in bus:
		try:
			msg_handler.handle_msg(msg.arbitration_id, msg.data)
		except:
			print("\n\n=========== ERROR ============")
			print(traceback.format_exc())
			send_msg(bus, msg_disable_system)
			break
