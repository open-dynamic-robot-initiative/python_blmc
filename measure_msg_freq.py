"""
Measure the frequency of messages with the specified arbitration id.
"""
from __future__ import print_function
import sys
import can
import numpy as np
import blmc.can_helper as ch
from blmc.helper import get_time


BITRATE = 1e6


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage:\t {} arbitration_id".format(sys.argv[0]))
        sys.exit(1)

    arb_id = int(sys.argv[1], 0)

    print("Monitor messages with id {}".format(arb_id))

    bus = can.interface.Bus(bitrate=BITRATE)
    last_print = 0
    stemps = []

    ch.send_command(bus, ch.Command.send_all, 1)

    # wait for messages and update data
    for msg in bus:
        #canhndlr.handle_msg(msg.arbitration_id, msg.data)
        t = get_time()
        if msg.arbitration_id == arb_id:
            #stemps.append(t)
            stemps.append(msg.timestamp)

        if last_print < t - 1:
            if len(stemps) == 0:
                print("No messages")
                continue

            last_print = t
            astemp = np.array(stemps)
            dts = (astemp[1:] - astemp[:-1]) * 1000
            dts_mean = dts.mean()
            print("ID: {} | Avg. dt [ms]: {:.3f} | Avg. freq [Hz]: {:.1f} | dt std dev"
                    " [ms]: {:.3f} | min/max [ms]: {:.3f}/{:.3f}"
                    .format(arb_id, dts_mean, (1.0/dts_mean*1000), dts.std(),
                        dts.min(), dts.max()))
            # clear values
            stemps = []
