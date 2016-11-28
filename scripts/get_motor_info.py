#!/usr/bin/env python
"""
Small working example on how to access CAN bus.
"""
from __future__ import print_function
import can
import time
import blmc.motor_data as md
import blmc.conversion as cnv
from blmc.can_helper import Command, send_command


BITRATE = 1e6


if __name__ == "__main__":
    mtr_data = md.MotorData()
    adc = md.AdcResult()
    bus = can.interface.Bus(bitrate=BITRATE)

    canhndlr = md.MessageHandler()
    canhndlr.set_id_handler(md.ArbitrationIds.status, mtr_data.set_status)
    canhndlr.set_id_handler(md.ArbitrationIds.current, mtr_data.set_current)
    canhndlr.set_id_handler(md.ArbitrationIds.position, mtr_data.set_position)
    canhndlr.set_id_handler(md.ArbitrationIds.velocity, mtr_data.set_velocity)
    canhndlr.set_id_handler(md.ArbitrationIds.adc6, adc.set_values)

    send_command(bus, Command.send_all, 1)

    # wait for messages and update data
    last_print = 0
    for msg in bus:
        canhndlr.handle_msg(msg)

        t = time.time()
        if last_print < t - 1:
            last_print = t
            print("{}  |  {}".format(mtr_data.to_string(), adc.to_string()))
