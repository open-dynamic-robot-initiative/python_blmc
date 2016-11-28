#!/usr/bin/env python
"""
Example on how the get data from the OptoForce sensor via CAN.
"""
from __future__ import print_function
import os
import ctypes
import struct
import time
import can
import signal
import sys
from blmc.conversion import *
from blmc.optoforce import *
from blmc.motor_data import ArbitrationIds

BITRATE = 1e6


def handle_package(data):
    pkt = OptoForceDataPacket31()
    try:
        pkt.set_packet_bytes(data)
        total = (pkt.fx_N**2 + pkt.fy_N**2 + pkt.fz_N**2) ** (0.5)
        print("{}, total: {:.2f}".format(pkt.to_string(), total))
    except ValueError, e:
        print("Error: {}".format(e))


if __name__ == "__main__":
    bus = can.interface.Bus(bitrate=BITRATE)
    data = None
    last_package_t = 0

    print("Initialize OptoForce...")
    ofconf = OptoForceConfig()

    # first have to unzero so that we can rezero in the next step ??
    #ofconf.unzero()
    #ofconf.send_via_can(bus, ArbitrationIds.optoforce_recv)

    # new rezero and set desired frequency
    ofconf.zero()
    ofconf.set_sample_frequency(
           OptoForceConfig.SampleFreq.Hz_10)
    ofconf.send_via_can(bus, ArbitrationIds.optoforce_recv)

    # wait for messages and update data
    for msg in bus:
        arb_id = msg.arbitration_id
        if arb_id == ArbitrationIds.optoforce_trans:

            # check for header 0xAA07080A
            if msg.data[0:4] == b"\xAA\x07\x08\x0A": #[0xAA, 0x07, 0x08, 0x0A]:
                data = msg.data
            elif data is not None:
                t = time.time()
                data += msg.data
                #print((t-last_package_t)*1000)
                last_package_t = t
                handle_package(data)
                # clear after data is handled
                data = None
            else:
                print("Unexpected package {}".format(
                    [int(x) for x in msg.data]))
