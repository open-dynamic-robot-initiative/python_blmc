"""
Functions to pack/unpack data for/from CAN messages.
"""
import struct


QVAL = 2**24


def bytes_to_uint16(data):
	# "H" = unsigned short (2 bytes)
	# Note: bytes in data are in reversed order, so use data[::-1]
	return struct.unpack("H", data[::-1])[0]


def uint16_to_bytes(value):
	return struct.pack("H", value)[::-1]


def bytes_to_sint16(data):
	# "h" = signed short (2 bytes)
	# Note: bytes in data are in reversed order, so use data[::-1]
	return struct.unpack("h", data[::-1])[0]


def q_bytes_to_value(data):
	"""Convert bytes recevied via CAN to float value."""

	assert len(data) == 4
	# convert bytes to signed int (note: bytes in data are in reversed order)
	qval = struct.unpack("<i", data[::-1])[0]
	# convert q-value to human-readable float
	fval = float(qval) / QVAL
	return fval

def value_to_q_bytes(fval):
	qval = int(fval * QVAL)
	rdata = struct.pack("<i", qval)
	return rdata[::-1]


