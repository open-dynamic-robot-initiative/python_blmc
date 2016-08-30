"""
Small working example on how to access CAN bus.
"""
from __future__ import print_function
import os
import struct
import time
import can
import signal
import sys


QVAL = 2**24

#-------------------------------------------------------------------------------
# PID.py
# A simple implementation of a PID controller
#-------------------------------------------------------------------------------
# Example source code for the book "Real-World Instrumentation with Python"
# by J. M. Hughes, published by O'Reilly Media, December 2010,
# ISBN 978-0-596-80956-0.
# http://examples.oreilly.com/9780596809577/CH09/PID.py
#-------------------------------------------------------------------------------
class PID:
    """ Simple PID control.

        This class implements a simplistic PID control algorithm. When first
        instantiated all the gain variables are set to zero, so calling
        the method GenOut will just return zero.
    """
    def __init__(self):
        # initialze gains
        self.Kp = 0
        self.Kd = 0
        self.Ki = 0

        self.Initialize()

    def SetKp(self, invar):
        """ Set proportional gain. """
        self.Kp = invar

    def SetKi(self, invar):
        """ Set integral gain. """
        self.Ki = invar

    def SetKd(self, invar):
        """ Set derivative gain. """
        self.Kd = invar

    def SetPrevErr(self, preverr):
        """ Set previous error value. """
        self.prev_err = preverr

    def Initialize(self):
        # initialize delta t variables
        self.currtm = time.time()
        self.prevtm = self.currtm

        self.prev_err = 0

        # term result variables
        self.Cp = 0
        self.Ci = 0
        self.Cd = 0


    def GenOut(self, error):
        """ Performs a PID computation and returns a control value based on
            the elapsed time (dt) and the error signal from a summing junction
            (the error parameter).
        """
        self.currtm = time.time()               # get t
        dt = self.currtm - self.prevtm          # get delta t
        de = error - self.prev_err              # get delta error

        self.Cp = self.Kp * error               # proportional term
        self.Ci += error * dt                   # integral term

        self.Cd = 0
        if dt > 0:                              # no div by zero
            self.Cd = de/dt                     # derivative term

        self.prevtm = self.currtm               # save t for next pass
        self.prev_err = error                   # save t-1 error

        # sum the terms and return the result
        return self.Cp + (self.Ki * self.Ci) + (self.Kd * self.Cd)


msg_enable_motor1 = can.Message(arbitration_id=0x000,
		data=[0, 0, 0, 1, 0, 0, 0, 2],
		extended_id=False)

msg_disable_motor1 = can.Message(arbitration_id=0x000,
		data=[0, 0, 0, 0, 0, 0, 0, 2],
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


class MotorData:
	current = 0
	position = 0
	velocity = 0

	def set_current_pos(self, data):
		self.current = q_bytes_to_value(data[0:4])
		self.position = q_bytes_to_value(data[4:8])

	def set_velocity(self, data):
		self.velocity = q_bytes_to_value(data)

	def to_string(self):
		return "Iq: {:.3f},  Pos: {:.3f},  Speed: {:.3f}".format(
				self.current, self.position, self.velocity)


class Status:
	system_enabled = 0
	mtr1_enabled = 0
	mtr1_ready = 0
	mtr2_enabled = 0
	mtr2_ready = 0
	mtr1_overheat = 0
	mtr2_overheat = 0
	system_error = 0

	def set_status(self, data):
		status_code = data[0]
		self.system_enabled = status_code & 1
		self.mtr1_enabled = status_code & (1 << 1)
		self.mtr1_ready = status_code & (1 << 2)
		self.mtr2_enabled = status_code & (1 << 3)
		self.mtr2_enabled = status_code & (1 << 4)
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



class VelocityController:

	def __init__(self, bus, Kp, Ki):
		self._bus = bus
		self._mtr1 = MotorData()
		self._status = Status()
		self._pid = PID()
		self._maxval = 9.0
		self._iqref = 0

		self._pid.SetKp(Kp)
		self._pid.SetKi(Ki)

	def run(self, refspeed):
		if self._status.mtr1_enabled and self._status.mtr1_ready:
			error = refspeed - self._mtr1.velocity
			u = self._pid.GenOut(error)
			self._iqref = u

			# clamp to allowed range
			self._iqref = min(self._iqref, self._maxval)
			self._iqref = max(self._iqref, -self._maxval)

			print("Speed = {:.4f}\t\tIqRef = {:.4f}".format(
				self._mtr1.velocity, self._iqref))
			self.send_mtr1_current(self._iqref)

	def send_mtr1_current(self, iq_ref):
		data = [0, 0, 0, 0, 0, 0, 0, 0]
		data[0:4] = value_to_q_bytes(iq_ref)

		msg = can.Message(arbitration_id=0x005,
				data=data,
				extended_id=False)
		send_msg(self._bus, msg)




if __name__ == "__main__":
	if len(sys.argv) != 4:
		print("Usage: {} goal_speed Kp Ki".format(sys.argv[0]))
		sys.exit(1)

	goal_speed = float(sys.argv[1])
	Kp = float(sys.argv[2])
	Ki = float(sys.argv[3])

	bus = can.interface.Bus(bitrate=250000)

	print("Setup controller with Kp = {}, Ki = {}".format(Kp, Ki))
	print("Goal speed: {}".format(goal_speed))
	vctrl = VelocityController(bus, Kp, Ki)

	# setup sigint handler to disable motor on CTRL+C
	def sigint_handler(signal, frame):
			print('Stop motor and shut down.')
			vctrl.send_mtr1_current(0)
			send_msg(bus, msg_disable_motor1)
			sys.exit(0)
	signal.signal(signal.SIGINT, sigint_handler)

	print("Enable motor...")
	vctrl.send_mtr1_current(0) # start with zero
	send_msg(bus, msg_enable_motor1)

	# wait for messages and update data
	for msg in bus:
		arb_id = msg.arbitration_id
		if arb_id == 0x010:
			vctrl._status.set_status(msg.data)
		elif arb_id == 0x021:
			vctrl._mtr1.set_current_pos(msg.data)
		elif arb_id == 0x031:
			vctrl._mtr1.set_velocity(msg.data)

			# emergency break
			if vctrl._mtr1.velocity > goal_speed * 3:
				send_msg(bus, msg_disable_system)
				print("Too fast! EMERGENCY BREAK!")
				break

			# trigger vel ctrl everytime we get a new velocity
			vctrl.run(goal_speed)

		#print("\r{}   \tMTR1: {}   \tMTR2: {}".format(
		#	status.to_string(), mtr1.to_string(), mtr2.to_string()),)
