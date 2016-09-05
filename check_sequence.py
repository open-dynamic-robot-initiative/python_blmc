"""
Small working example on how to access CAN bus.
"""
from __future__ import print_function
import os
import struct
import time
import can
import signal
import sys

BITRATE = 1e6
QVAL = 2**24


def bytes_to_value(data):
	"""Convert bytes recevied via CAN to float value."""

	assert len(data) == 4
	# convert bytes to signed int (note: bytes in data are in reversed order)
	val = struct.unpack("<I", data[::-1])[0]
	return val

def value_to_q_bytes(fval):
	qval = int(fval * QVAL)
	rdata = struct.pack("<i", qval)
	return rdata[::-1]



if __name__ == "__main__":
	bus = can.interface.Bus(bitrate=BITRATE)

	pos_start = None
	vel_start = None
	pos_seq_num = None
	vel_seq_num = None
	pos_miss = 0
	vel_miss = 0
	last_pos_seqno = None
	last_vel_seqno = None

	# setup sigint handler to disable motor on CTRL+C
	def sigint_handler(signal, frame):
		print("Position Sequence Length: {}".format(pos_seq_num - pos_start))
		print("Velocity Sequence Length: {}".format(vel_seq_num - vel_start))
		print("Missed position frames: {}".format(pos_miss))
		print("Missed velocity frames: {}".format(vel_miss))
		sys.exit(0)
	signal.signal(signal.SIGINT, sigint_handler)

	print("Start listening. Press CTRL+C to stop and show result.")
	# wait for messages and update data
	for msg in bus:
		arb_id = msg.arbitration_id
		if arb_id == 0x021:
			seqno = bytes_to_value(msg.data[4:])
			#print("pos: {}".format(seqno))
			if pos_seq_num is None:
				pos_start = seqno
				pos_seq_num = seqno

			if seqno != pos_seq_num:
				#print("{} != {}".format(seqno, pos_seq_num))
				print("pos: {} --> {}".format(last_pos_seqno, seqno))
				pos_miss += seqno - pos_seq_num
			pos_seq_num = seqno + 1
			last_pos_seqno = seqno
		elif arb_id == 0x031:
			seqno = bytes_to_value(msg.data)
			#print("vel: {}".format(seqno))
			if vel_seq_num is None:
				vel_start = seqno
				vel_seq_num = seqno
			if seqno != vel_seq_num:
				print("vel: {} --> {}".format(last_vel_seqno, seqno))
				vel_miss += seqno - vel_seq_num
			vel_seq_num = seqno + 1
			last_vel_seqno = seqno
