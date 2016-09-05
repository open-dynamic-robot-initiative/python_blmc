"""
Small working example on how to access CAN bus.
"""
from __future__ import print_function
import os
import ctypes
import struct
import time
import can
import signal
import sys

BITRATE = 1e6


def value_to_q_bytes(fval):
	qval = int(fval * QVAL)
	rdata = struct.pack("<i", qval)
	return rdata[::-1]


def bytes_to_uint16(data):
	# "H" = unsigned short (2 bytes)
	# Note: bytes in data are in reversed order, so use data[::-1]
	return struct.unpack("H", data[::-1])[0]


def bytes_to_sint16(data):
	# "h" = signed short (2 bytes)
	# Note: bytes in data are in reversed order, so use data[::-1]
	return struct.unpack("h", data[::-1])[0]


class OptoForceStatusBits(ctypes.LittleEndianStructure):
	_fields_ = [
			("sensor_number", ctypes.c_uint16, 3),    # bit 0:2
			("multiple_sensors", ctypes.c_uint16, 1), # bit 3
			("overload_tz", ctypes.c_uint16, 1),      # bit 4
			("overload_ty", ctypes.c_uint16, 1),      # bit 5
			("overload_tx", ctypes.c_uint16, 1),      # bit 6
			("overload_fz", ctypes.c_uint16, 1),      # bit 7
			("overload_fy", ctypes.c_uint16, 1),      # bit 8
			("overload_fx", ctypes.c_uint16, 1),      # bit 9
			("sensor_error", ctypes.c_uint16, 3),     # bit 10:12
			("daq_error", ctypes.c_uint16, 3),        # bit 13:15
			]


class OptoForceStatus(ctypes.Union):
	_fields_ = [
			("bits", OptoForceStatusBits),
			("all", ctypes.c_uint16)
			]
	_anonymous_ = ("bits",)


	class DaqErrorType:
		NO_ERROR = 0
		DAQ_ERROR = 1
		COMMUNICATION_ERROR = 2

	class SensorErrorType:
		NO_ERROR = 0
		SENSOR_NOT_DETECTED = 1
		SENSOR_FAILURE = 2
		TEMPERATURE_ERROR = 4


class OptoForceDataPacket31:

	def __init__(self):
		self._header = b"\xAA\x07\x08\x0A"
		self.sample_counter = None
		self.status = None
		self.fx = None
		self.fy = None
		self.fz = None

	def set_packet_bytes(self, data):
		"""Decode a packet and store the values to the object."""
		if len(data) != 16:
			raise ValueError("Invalid packet size")

		header = data[0:4]
		if header != self._header:
			raise ValueError("Invalid packet header")

		checksum = bytes_to_uint16(data[14:16])
		# checksum = sum of all preceding bytes, including header
		if checksum != sum(data[:14]):
			raise ValueError("Invalid checksum")

		self.sample_counter = bytes_to_uint16(data[4:6])
		self.status = OptoForceStatus()
		self.status.all = bytes_to_uint16(data[6:8])
		self.fx = bytes_to_sint16(data[8:10])
		self.fy = bytes_to_sint16(data[10:12])
		self.fz = bytes_to_sint16(data[12:14])

	def to_string(self):
		return "x: {}, y: {}, z: {}".format(self.fx, self.fy, self.fz)



def handle_package(data):
	pkt = OptoForceDataPacket31()
	try:
		pkt.set_packet_bytes(data)
		print(pkt.to_string())
	except ValueError, e:
		print("Error: {}".format(e))



if __name__ == "__main__":
	bus = can.interface.Bus(bitrate=BITRATE)

	data = None
	last_package_t = 0

	# wait for messages and update data
	for msg in bus:
		arb_id = msg.arbitration_id
		if arb_id == 0x100:
			#print([hex(x) for x in msg.data])
			
			# check for header 0xAA07080A
			if msg.data[0:4] == b"\xAA\x07\x08\x0A": #[0xAA, 0x07, 0x08, 0x0A]:
				data = msg.data
			elif data is not None:
				t = time.time()
				data += msg.data
				print((t-last_package_t)*1000)
				last_package_t = t
				handle_package(data)
				# clear after data is handled
				data = None
			else:
				print("Unexpected package {}".format(
					[int(x) for x in msg.data]))

