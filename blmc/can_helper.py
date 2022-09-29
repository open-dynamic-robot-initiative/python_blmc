"""
Some simple helper functions for sending CAN messages.
"""
import can
from .conversion import value_to_q_bytes
from .motor_data import ArbitrationIds, init_position_offset
from .helper import get_time


# TODO: this should be in motor_data ?!
class Command:
    enable_sys = 1
    enable_mtr1 = 2
    enable_mtr2 = 3
    enable_vspring1 = 4
    enable_vspring2 = 5
    send_current = 12
    send_position = 13
    send_velocity = 14
    send_adc6 = 15
    send_all = 20


def send_command(bus, cmd_id, value):
    # As long as both command id and value do not exceed a single byte, we can
    # keep it that simple :)
    # FIXME make better anyway, otherwise it will break in the future!
    msg = can.Message(
        arbitration_id=ArbitrationIds.command,
        data=[0, 0, 0, value, 0, 0, 0, cmd_id],
        is_extended_id=False,
    )
    send_msg(bus, msg)


def send_msg(bus, msg):
    """Send a single message on the CAN bus."""
    try:
        bus.send(msg)
        # print("Message sent on {}".format(bus.channel_info))
    except can.CanError:
        print("Message NOT sent")


def send_mtr_current(bus, mtr1_iqref, mtr2_iqref):
    data = [0, 0, 0, 0, 0, 0, 0, 0]
    data[0:4] = value_to_q_bytes(mtr1_iqref)
    data[4:8] = value_to_q_bytes(mtr2_iqref)

    msg = can.Message(arbitration_id=0x005, data=data, is_extended_id=False)
    send_msg(bus, msg)


def update_position(bus, mtr_data, wait_period):
    """Update the motor position

    Fetch CAN messages for the give period and using the last one to update the
    motor positions.  This is necessary as it seems that the python can module
    latches some old, outdated messages.
    """
    start = get_time()
    for msg in bus:
        if msg.arbitration_id == ArbitrationIds.position:
            if start < get_time() - wait_period:
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


def start_system(bus, mtr_data, init_position=True):
    send_command(bus, Command.enable_sys, 1)
    send_mtr_current(bus, 0, 0)  # clear old values
    send_command(bus, Command.send_all, 1)
    send_command(bus, Command.enable_mtr1, 1)
    send_command(bus, Command.enable_mtr2, 1)

    wait_for_motors_ready(bus, mtr_data)
    if init_position:
        init_position_offset(bus, mtr_data)


def stop_system(bus):
    send_command(bus, Command.enable_sys, 0)
    send_mtr_current(bus, 0, 0)  # clear old values
