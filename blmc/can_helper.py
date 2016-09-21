"""
Some simple helper functions for sending CAN messages.
"""
import can
import time
from .conversion import value_to_q_bytes
from .motor_data import ArbitrationIds

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


def update_position(bus, mtr_data, wait_period):
    """Update the motor position

    Fetch CAN messages for the give period and using the last one to update the
    motor positions.  This is necessary as it seems that the python can module
    latches some old, outdated messages.
    """
    start = time.clock()
    for msg in bus:
        if msg.arbitration_id == ArbitrationIds.position:
            if start < time.clock() - wait_period:
                mtr_data.set_position(msg)
                return

def wait_for_motors_ready(bus, mtr_data):
    """Block until both motors are ready."""
    printed_align_msg = False
    for msg in bus:
        if msg.arbitration_id == ArbitrationIds.status:
            mtr_data.set_status(msg)
            if mtr_data.status.mtr1_ready and mtr_data.status.mtr2_ready:
                return
            elif not printed_align_msg:
                print("Motors are aligned. Please wait...")
                printed_align_msg = True
