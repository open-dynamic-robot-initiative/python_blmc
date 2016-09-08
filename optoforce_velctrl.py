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
from blmc.can_helper import *
from blmc.controllers import VelocityController

BITRATE = 1e6


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

	of_packet_receiver = OptoForcePacketReceiver()
	optofullscale = 1000.0
	optospeed = 0

	mtr_data = MotorData()
	bus = can.interface.Bus(bitrate=BITRATE)

	print("Setup controller with Kp = {}, Ki = {}".format(Kp, Ki))
	print("Goal speed: {}".format(goal_speed))
	vctrl1 = VelocityController(Kp, Ki, 0)
	vctrl2 = VelocityController(Kp, Ki, 0)

	# setup sigint handler to disable motor on CTRL+C
	def sigint_handler(signal, frame):
			print('Stop motor and shut down.')
			send_mtr_current(bus, 0, 0)
			send_msg(bus, msg_disable_motor1)
			send_msg(bus, msg_disable_motor2)
			sys.exit(0)
	signal.signal(signal.SIGINT, sigint_handler)

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

	# wait a second for the initial messages to be handled
	time.sleep(0.2)

	def on_velocity_msg(msg):
		mtr_data.set_velocity(msg)
		max_speed = goal_speed * 3

		# emergency break
		if ((mtr_data.mtr1.velocity.value > max_speed)
				or (mtr_data.mtr2.velocity.value > max_speed)):
			send_msg(bus, msg_disable_system)
			print(mtr_data.to_string())
			print("Motor too fast! EMERGENCY BREAK!")
			sys.exit(0)

		if mtr_data.status.mtr1_ready:
			vctrl1.update_data(mtr_data.mtr1)
			vctrl1.run(optospeed)

		if mtr_data.status.mtr2_ready:
			vctrl2.update_data(mtr_data.mtr2)
			vctrl2.run(optospeed)

		print(mtr_data.to_string())
		send_mtr_current(bus, vctrl1.iqref, vctrl2.iqref)

	def on_optoforce_msg(msg):
		global optospeed

		ofpkt = of_packet_receiver.receive_frame(msg.data)
		if ofpkt is not None:
			optospeed = float(max(0, ofpkt.fz)) / optofullscale * goal_speed

	msg_handler = MessageHandler()
	msg_handler.set_id_handler(ArbitrationIds.status, mtr_data.set_status)
	msg_handler.set_id_handler(ArbitrationIds.current, mtr_data.set_current)
	msg_handler.set_id_handler(ArbitrationIds.position, mtr_data.set_position)
	msg_handler.set_id_handler(ArbitrationIds.velocity, on_velocity_msg)
	#msg_handler.set_id_handler(0x050, mtr_data.set_status)
	msg_handler.set_id_handler(ArbitrationIds.optoforce_trans, on_optoforce_msg)

	# wait for messages and update data
	for msg in bus:
		try:
			msg_handler.handle_msg(msg)
		except:
			print("\n\n=========== ERROR ============")
			print(traceback.format_exc())
			send_msg(bus, msg_disable_system)
			break
