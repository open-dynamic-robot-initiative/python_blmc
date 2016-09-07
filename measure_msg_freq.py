"""
Measure the frequency of messages with the specified arbitration id.
"""
from __future__ import print_function
import sys
import can
import time
import numpy as np


BITRATE = 1e6


if __name__ == "__main__":
	if len(sys.argv) < 2:
		print("Usage:\t {} arbitration_id".format(sys.argv[0]))
		sys.exit(1)
	
	arb_id = int(sys.argv[1], 0)

	print("Monitor messages with id {}".format(arb_id))

	bus = can.interface.Bus(bitrate=BITRATE)

	#canhndlr = md.MessageHandler()
	#canhndlr.set_id_handler(md.ArbitrationIds.status, mtr_data.set_status)
	#canhndlr.set_id_handler(md.ArbitrationIds.current, mtr_data.set_current)
	#canhndlr.set_id_handler(md.ArbitrationIds.position, mtr_data.set_position)
	#canhndlr.set_id_handler(md.ArbitrationIds.velocity, mtr_data.set_velocity)
	#canhndlr.set_id_handler(md.ArbitrationIds.adc6, adc.set_values)

	last_print = 0
	stemps = []

	# wait for messages and update data
	for msg in bus:
		#canhndlr.handle_msg(msg.arbitration_id, msg.data)
		t = time.clock()
		if msg.arbitration_id == arb_id:
			stemps.append(t)

		if last_print < t - 1:
			if len(stemps) == 0:
				print("No messages")
				continue

			last_print = t
			astemp = np.array(stemps)
			dts = (astemp[1:] - astemp[:-1]) * 1000
			dts_mean = dts.mean()
			print("ID: {} | Avg. dt [ms]: {:.3f} | Avg. freq [Hz]: {:.1f} | dt std dev"
					" [ms]: {:.3f}"
					.format(arb_id, dts_mean, (1.0/dts_mean*1000), dts.std()))
			# clear values
			stemps = []
