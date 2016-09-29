"""
Helper classes to decode data sent by the OptoForce sensor.
"""
import ctypes
import can
from . import conversion as cnv

# OptoForce OMD-20-SE-40N, S/N: ISE0A129
# 40 N @ 16037 counts
def counts_to_newton(counts):
    return counts / 16037. * 40.


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
        self.fx_N = None
        self.fy_N = None
        self.fz_N = None

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
            raise ValueError("Invalid checksum. Packet: {}".format([hex(x) for x in data]))

        self.sample_counter = cnv.bytes_to_uint16(data[4:6])
        self.status = OptoForceStatus()
        self.status.all = cnv.bytes_to_uint16(data[6:8])
        self.fx = cnv.bytes_to_sint16(data[8:10])
        self.fy = cnv.bytes_to_sint16(data[10:12])
        self.fz = cnv.bytes_to_sint16(data[12:14])

        self.fx_N = counts_to_newton(self.fx)
        self.fy_N = counts_to_newton(self.fy)
        self.fz_N = counts_to_newton(self.fz)

    def to_string(self):
        #return "x: {}, y: {}, z: {}".format(self.fx, self.fy, self.fz)
        return "x: {:.3f} N, y: {:.3f} N, z: {:.3f} N".format(self.fx_N, self.fy_N, self.fz_N)


class OptoForceConfig:

    class SampleFreq:
        STOP = 0
        Hz_1000 = 1
        Hz_333 = 3
        Hz_100 = 10 # default
        Hz_30 = 33
        Hz_10 = 100

    class FilterFreq:
        NO_FILTER = 0
        Hz_500 = 1
        Hz_150 = 2
        Hz_50 = 3
        Hz_15 = 4 # default
        Hz_5 = 5
        Hz_1_5 = 6 # 1.5 Hz

    class SetZero:
        ZERO = 255
        UNZERO = 0

    def __init__(self):
        self._header = b"\xAA\x00\x32\x03"
        self._speed = OptoForceConfig.SampleFreq.Hz_100
        self._filter = OptoForceConfig.FilterFreq.Hz_15
        self._zero = OptoForceConfig.SetZero.UNZERO

    def set_sample_frequency(self, sample_freq_code):
        self._speed = sample_freq_code

    def set_filter_frequency(self, filter_freq_code):
        self._filter = filter_freq_code

    def zero(self):
        self._zero = OptoForceConfig.SetZero.ZERO

    def unzero(self):
        self._zero = OptoForceConfig.SetZero.UNZERO

    def calc_checksum(self):
        return 170 + 0 + 50 + 3 + self._speed + self._filter + self._zero

    def get_can_bytes(self):
        data = [0 for i in range(9)]
        data[0:4] = self._header
        data[4] = self._speed
        data[5] = self._filter
        data[6] = self._zero
        data[7:9] = cnv.uint16_to_bytes(self.calc_checksum())

        return (bytearray(data[0:8]), bytearray(data[8]))

    def send_via_can(self, bus, arbitration_id=0x101):
        data = self.get_can_bytes()
        msg1 = can.Message(
                arbitration_id = arbitration_id,
                data = data[0],
                extended_id=False)
        msg2 = can.Message(
                arbitration_id = arbitration_id,
                data = data[1],
                extended_id=False)

        try:
            bus.send(msg1)
            bus.send(msg2)
        except can.CanError:
            print("OptoForce Config Message NOT sent")


class OptoForcePacketReceiver:

    def __init__(self):
        self._pkt_bytes = None
        self.data = None

    def receive_frame(self, fdata):
        # check for header 0xAA07080A
        if fdata[0:4] == b"\xAA\x07\x08\x0A": #[0xAA, 0x07, 0x08, 0x0A]:
            self._pkt_bytes = fdata
            return None

        elif self._pkt_bytes is not None:
            # if not beginning with header and we already have the first part
            self._pkt_bytes += fdata

            # decode data
            pkt = OptoForceDataPacket31()
            pkt.set_packet_bytes(self._pkt_bytes)

            # clear after data is handled
            self._pkt_bytes = None

            self.data = pkt

        else:
            # if not beginning with header but we dont already have the first
            # part
            print("Unexpected package {}".format(
                [int(x) for x in fdata]))
            return None
