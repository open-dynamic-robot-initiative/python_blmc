#!/usr/bin/env python
"""
Simple Velocity Controller for Motor 1.

This is a small example on how to access CAN bus.  It controls the velocity of
motor 1 with a simple PI controller.  The velocity reference is given by a
potentiometer that is connected to ADCINA6.
"""

import time
import can
import signal
import sys
import traceback
from blmc.motor_data import MotorData, AdcResult, MessageHandler, ArbitrationIds
from blmc.controllers import VelocityController
from blmc.can_helper import send_mtr_current

BITRATE = 1e6


msg_enable_motor1 = can.Message(
    arbitration_id=0x000, data=[0, 0, 0, 1, 0, 0, 0, 2], is_extended_id=False
)

msg_disable_motor1 = can.Message(
    arbitration_id=0x000, data=[0, 0, 0, 0, 0, 0, 0, 2], is_extended_id=False
)

msg_ensable_system = can.Message(
    arbitration_id=0x000, data=[0, 0, 0, 1, 0, 0, 0, 1], is_extended_id=False
)

msg_disable_system = can.Message(
    arbitration_id=0x000, data=[0, 0, 0, 0, 0, 0, 0, 1], is_extended_id=False
)


def send_msg(bus, msg):
    """Send a single message on the CAN bus."""
    try:
        bus.send(msg)
        # print("Message sent on {}".format(bus.channel_info))
    except can.CanError:
        print("Message NOT sent")


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: {} max_speed Kp Ki".format(sys.argv[0]))
        sys.exit(1)

    goal_speed = float(sys.argv[1])
    Kp = float(sys.argv[2])
    Ki = float(sys.argv[3])

    mtr_data = MotorData()
    adc = AdcResult()
    bus = can.interface.Bus(bitrate=BITRATE)

    # setup sigint handler to disable motor on CTRL+C
    def sigint_handler(signal, frame):
        print("Stop motor and shut down.")
        send_mtr_current(bus, 0, 0)
        send_msg(bus, msg_disable_motor1)
        sys.exit(0)

    signal.signal(signal.SIGINT, sigint_handler)

    print("Setup controller with Kp = {}, Ki = {}".format(Kp, Ki))
    print("Goal speed: {}".format(goal_speed))
    vctrl = VelocityController(bus, Kp, Ki, 0)

    print("Enable system...")
    send_msg(bus, msg_ensable_system)

    print("Enable motor...")
    send_mtr_current(bus, 0, 0)  # start with zero
    send_msg(bus, msg_enable_motor1)

    # wait a second for the initial messages to be handled
    time.sleep(0.2)

    log_data = []

    def on_velocity_msg(msg):
        mtr_data.set_velocity(msg)

        log_data.append(mtr_data.mtr1.velocity.value)

        # emergency break
        # if mtr_data.mtr1.velocity.value > goal_speed * 3:
        #     send_msg(bus, msg_disable_system)
        #     print("Too fast! EMERGENCY BREAK!")
        #     sys.exit(0)

        if mtr_data.status.mtr1_ready:
            vctrl.update_data(mtr_data.mtr1)
            # vctrl.run(goal_speed * adc.a)
            vctrl.run(goal_speed)

            send_mtr_current(bus, vctrl.iqref, 0)
        else:
            send_mtr_current(bus, 0, 0)
        print()

    msg_handler = MessageHandler()
    msg_handler.set_id_handler(ArbitrationIds.status, mtr_data.set_status)
    msg_handler.set_id_handler(ArbitrationIds.current, mtr_data.set_current)
    msg_handler.set_id_handler(ArbitrationIds.position, mtr_data.set_position)
    msg_handler.set_id_handler(ArbitrationIds.velocity, on_velocity_msg)
    msg_handler.set_id_handler(ArbitrationIds.adc6, adc.set_values)

    start = time.time()

    # wait for messages and update data
    for msg in bus:
        try:
            msg_handler.handle_msg(msg)
        except Exception:
            print("\n\n=========== ERROR ============")
            print(traceback.format_exc())
            send_msg(bus, msg_disable_system)
            break

        if time.time() > start + 15:
            send_msg(bus, msg_disable_system)
            break

    # plot logged data
    import matplotlib.pyplot as plt
    plt.axhline(y=goal_speed, color='r', linestyle='-')
    plt.plot(log_data)
    plt.show()
