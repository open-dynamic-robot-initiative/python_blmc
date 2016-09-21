"""
Helper classes to interpret data send by the board
"""
import time
import cPickle as pickle
from .conversion import *


class ArbitrationIds:
    status = 0x010
    current = 0x020
    position = 0x030
    velocity = 0x040
    adc6 = 0x050
    optoforce_trans = 0x100
    optoforce_recv  = 0x101

    command = 0x000
    current_ref = 0x005


class MessageHandler:

    def __init__(self):
        self._id_fnx_map = {}

    def set_id_handler(self, arbitration_id, func):
        self._id_fnx_map[arbitration_id] = func

    def handle_msg(self, msg):
        if msg.arbitration_id in self._id_fnx_map:
            return self._id_fnx_map[msg.arbitration_id](msg)
        else:
            return None


class StampedValue:
    def __init__(self, value=0, timestamp=0):
        self.value = value
        self.timestamp = timestamp


class MotorDataStruct:
    current = StampedValue()
    raw_position = StampedValue()
    position = StampedValue()
    velocity = StampedValue()

    def to_string(self):
        return "Iq: {:.3f},  Pos: {:.3f},  Speed: {:.3f}".format(
                self.current.value, self.position.value, self.velocity.value)


def MDL(data):
    return data[0:4]


def MDH(data):
    return data[4:8]


def _conv_stamp_2val_q_msg(msg):
    return (StampedValue(q_bytes_to_value(MDL(msg.data)), msg.timestamp),
            StampedValue(q_bytes_to_value(MDH(msg.data)), msg.timestamp))


class MotorData:
    def __init__(self):
        self.mtr1 = MotorDataStruct()
        self.mtr2 = MotorDataStruct()
        self.status = Status()
        self._zero_pos_offset = (0., 0.)

    def set_zero_position_offset(self, offset, apply_immediately=True):
        self._zero_pos_offset = offset
        if apply_immediately:
            self.mtr1.position.value -= self._zero_pos_offset[0]
            self.mtr2.position.value -= self._zero_pos_offset[1]

    def set_status(self, msg):
        self.status.set_status(msg)

    def set_current(self, msg):
        (self.mtr1.current, self.mtr2.current) = _conv_stamp_2val_q_msg(msg)

    def set_position(self, msg):
        (self.mtr1.raw_position, self.mtr2.raw_position) = \
                _conv_stamp_2val_q_msg(msg)
        (self.mtr1.position, self.mtr2.position) = \
                _conv_stamp_2val_q_msg(msg)

        # subtract zero position offset from raw position
        self.mtr1.position.value -= self._zero_pos_offset[0]
        self.mtr2.position.value -= self._zero_pos_offset[1]

    def set_velocity(self, msg):
        (self.mtr1.velocity, self.mtr2.velocity) = _conv_stamp_2val_q_msg(msg)

    def to_string(self):
        return "{} | MTR1: {}\t | MTR2: {}".format(
                self.status.to_string(),
                self.mtr1.to_string(),
                self.mtr2.to_string())


class Status:
    system_enabled = 0
    mtr1_enabled = 0
    mtr1_ready = 0
    mtr2_enabled = 0
    mtr2_ready = 0
    mtr1_overheat = 0
    mtr2_overheat = 0
    system_error = 0
    timestamp = 0

    def set_status(self, msg):
        self.timestamp = msg.timestamp
        status_code = msg.data[0]
        self.system_enabled = status_code & 1
        self.mtr1_enabled = status_code & (1 << 1)
        self.mtr1_ready = status_code & (1 << 2)
        self.mtr2_enabled = status_code & (1 << 3)
        self.mtr2_ready = status_code & (1 << 4)
        self.mtr1_overheat = status_code & (1 << 5)
        self.mtr2_overheat = status_code & (1 << 6)
        self.system_error = status_code & (1 << 7)

    def to_string(self):
        str_sys = "SYS: "
        str_sys += "E" if self.system_enabled else "D"
        str_sys += "!!!" if self.system_error else ""

        str_mtr1 = "MTR1: "
        str_mtr1 += "E" if self.mtr1_enabled else "D"
        str_mtr1 += "R" if self.mtr1_ready else "A"

        str_mtr2 = "MTR2: "
        str_mtr2 += "E" if self.mtr2_enabled else "D"
        str_mtr2 += "R" if self.mtr2_ready else "A"

        return "{} | {} | {}".format(str_sys, str_mtr1, str_mtr2)


class AdcResult:
    a = 0
    b = 0
    timestamp = 0
    filter_val = 0.002

    def set_values(self, msg):
        new_a = q_bytes_to_value(MDL(msg.data))
        new_b = q_bytes_to_value(MDH(msg.data))

        self.timestamp = msg.timestamp
        if abs(self.a - new_a) > self.filter_val:
            self.a = new_a
        if abs(self.b - new_b) > self.filter_val:
            self.b = new_b

    def to_string(self):
        return "ADC: {:.3f} / {:.3f}".format(self.a, self.b)


def init_position_offset(bus, mtr_data, calibration_file="calibration_data.p"):
    """
    Initialize the position offset to the zero position of the leg.

    This should always be done in the initialization phase, as the raw zero
    position of the joints is not based on absolute values but simple set to
    the position the leg has, when the MCU is reset.
    """
    # Offsets between positive stop and zero for the current leg:
    stop_offsets = pickle.load(open(calibration_file, "rb"))
    zero_offsets = None

    raw_input("Move both joints in positive direction until stop."
              " Then press enter")
    start = time.clock()
    for msg in bus:
        if msg.arbitration_id == ArbitrationIds.position:
            if start < time.clock() - 1:
                mtr_data.set_position(msg)
                zero_offsets = (mtr_data.mtr1.position.value - stop_offsets[0],
                                mtr_data.mtr2.position.value - stop_offsets[1])
                print("Okay, now release the leg.")
                mtr_data.set_zero_position_offset(zero_offsets)
                break


