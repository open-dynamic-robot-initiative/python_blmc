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
import numpy as np
from blmc.optoforce import *
from blmc.motor_data import *
from blmc.conversion import *
from blmc.controllers import PositionController

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


def send_mtr_current(bus, mtr1_iqref, mtr2_iqref):
	data = [0, 0, 0, 0, 0, 0, 0, 0]
	data[0:4] = value_to_q_bytes(mtr1_iqref)
	data[4:8] = value_to_q_bytes(mtr2_iqref)

	msg = can.Message(arbitration_id=0x005,
			data=data,
			extended_id=False)
	send_msg(bus, msg)


def get_position_reference(t):
	# range of motor 1: +/- 1.0
	# range of motor 2: +/- 1.1
	# start both at 0 and run with same frequency (i.e. they always are at zero
	# at the same time)

	range1 = 0.4
	range2 = 0.8
	freq = 0.8

	t_scale = 2.0 * np.pi * freq
	t = t * t_scale

	s1 = range1 * np.sin(t)
	s2 = range2 * np.sin(t)

	return (s1, s2)


def emergency_break(bus):
	send_mtr_current(bus, 0, 0)
	send_msg(bus, msg_disable_system)
	print("EMERGENCY STOP")
	sys.exit(0)


if __name__ == "__main__":
	#if len(sys.argv) != 8:
	#	print("Usage: {} Kp1 Ki1 Kd1 Kp2 Ki2 Kd2".format(sys.argv[0]))
	#	sys.exit(1)


	if len(sys.argv) == 8:
		Kp1 = float(sys.argv[2])
		Ki1 = float(sys.argv[3])
		Kd1 = float(sys.argv[4])
		Kp2 = float(sys.argv[5])
		Ki2 = float(sys.argv[6])
		Kd2 = float(sys.argv[7])
	else:
		print("Use default controller values")
		(Kp1, Ki1, Kd1) = (20, 0, 0.12)
		(Kp2, Ki2, Kd2) = (10, 0, 0.1)

	bus = can.interface.Bus(bitrate=BITRATE)

	# setup sigint handler to disable motor on CTRL+C
	def sigint_handler(signal, frame):
			print('Stop motor and shut down.')
			send_mtr_current(bus, 0, 0)
			send_msg(bus, msg_disable_motor1)
			send_msg(bus, msg_disable_motor2)
			sys.exit(0)
	signal.signal(signal.SIGINT, sigint_handler)


	mtr_data = MotorData()
	adc = AdcResult()
	pos_ctrl1 = PositionController(Kp1, Ki1, Kd1)
	pos_ctrl2 = PositionController(Kp2, Ki2, Kd2)
	pos_offset = None
	t_offset = None

	print("Setup controller 1 with Kp = {}, Ki = {}, Kd = {}".format(
		Kp1, Ki1, Kd1))
	print("Setup controller 2 with Kp = {}, Ki = {}, Kd = {}".format(
		Kp2, Ki2, Kd2))

	print("Enable system...")
	send_msg(bus, msg_ensable_system)

	print("Enable motors...")
	send_mtr_current(bus, 0, 0) # start with zero
	send_msg(bus, msg_enable_motor1)
	send_msg(bus, msg_enable_motor2)

	# wait a moment for the initial messages to be handled
	time.sleep(0.5)

	def on_position_msg(data):
		global pos_offset, t_offset

		mtr_data.set_position(data)

		if ((abs(mtr_data.mtr1.position) > 1.1)
				or (abs(mtr_data.mtr2.position) > 1.1)):
			emergency_break(bus)

		if pos_offset is None:
			pos_offset = (mtr_data.mtr1.position, mtr_data.mtr2.position)

		print(mtr_data.to_string())
		print(adc.to_string())

		if mtr_data.status.mtr1_ready and mtr_data.status.mtr2_ready:
			t = time.clock()
			if t_offset is None:
				t_offset = t
				# TODO check if position is close to zero at start
			t = t - t_offset
			# wait a moment before starting the sinus by using an additional
			# offset
			start_up_offset = 3
			if t > start_up_offset:
				t = t - start_up_offset

				(posref1, posref2) = get_position_reference(t)
				posref1 += pos_offset[0]
				posref2 += pos_offset[1]


				pos_ctrl1.update_data(mtr_data.mtr1)
				pos_ctrl1.run(posref1)
				pos_ctrl2.update_data(mtr_data.mtr2)
				pos_ctrl2.run(posref2)

			send_mtr_current(bus, pos_ctrl1.iqref, pos_ctrl2.iqref)
		else:
			send_mtr_current(bus, 0, 0)

		print()

	msg_handler = MessageHandler()
	msg_handler.set_id_handler(ArbitrationIds.status, mtr_data.set_status)
	msg_handler.set_id_handler(ArbitrationIds.current, mtr_data.set_current)
	msg_handler.set_id_handler(ArbitrationIds.position, on_position_msg)
	msg_handler.set_id_handler(ArbitrationIds.velocity, mtr_data.set_velocity)
	msg_handler.set_id_handler(ArbitrationIds.adc6, adc.set_values)

	# wait for messages and update data
	for msg in bus:
		try:
			msg_handler.handle_msg(msg.arbitration_id, msg.data)
		except:
			print("\n\n=========== ERROR ============")
			print(traceback.format_exc())
			send_msg(bus, msg_disable_system)
			break
