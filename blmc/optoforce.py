"""
Helper classes to decode data sent by the OptoForce sensor.
"""
import ctypes
from . import conversion as cnv

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

		checksum = cnv.bytes_to_uint16(data[14:16])
		# checksum = sum of all preceding bytes, including header
		if checksum != sum(data[:14]):
			raise ValueError("Invalid checksum")

		self.sample_counter = cnv.bytes_to_uint16(data[4:6])
		self.status = OptoForceStatus()
		self.status.all = cnv.bytes_to_uint16(data[6:8])
		self.fx = cnv.bytes_to_sint16(data[8:10])
		self.fy = cnv.bytes_to_sint16(data[10:12])
		self.fz = cnv.bytes_to_sint16(data[12:14])

	def to_string(self):
		return "x: {}, y: {}, z: {}".format(self.fx, self.fy, self.fz)


