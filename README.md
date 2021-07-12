Python BrushLess Motor Control
==============================


--------------------------------------------------------------------------------

**Note:** This package was developed some time ago using catkin/Python 2 and has
not been tested with Python 3 yet.

--------------------------------------------------------------------------------


The `blmc` package contains an API for the CAN interface of the motor control
microcontrollers of the Open Dynamic Robot Initiative as well as the OptoForce
sensor.

The scripts in the `scripts` directory use this API for several tasks like
evaluating the CAN interface, displaying motor parameters or running simple
velocity/position controllers.

This assumes that the
[mw_dual_motor_torque_ctrl](https://github.com/open-dynamic-robot-initiative/mw_dual_motor_torque_ctrl)
firmware is running on the board.


Requirements
============

* [python-can](https://python-can.readthedocs.io/en/latest/index.html)  
  _Note: When using on Windows with PCAN, make sure you have version >= 1.5.1_
* numpy
* LaunchPad with two motors, running the dual_motor_torque_ctrl program and
  connected to the computer via CAN.


Installation
============

1. Make sure python-can is installed and set up correctly (see documentation
   there).
    * The CAN interface and channel have to be set in the configuration file,
      see [python-can
      documentation](https://python-can.readthedocs.io/en/latest/configuration.html)
2. Some of the demo scripts need a prior calibration the offset between zero and
   stop position of the leg. Run the script `calibrate.py` (see below) to do
   this.
3. That's it. Simply launch the script you want to run.


About the Scripts
=================

There are a bunch of scripts that can be used for fast analysing and testing of
a motor.  At the same time these scripts serve as examples on how to use the CAN
API in the `blmc` Python module (at least for now, these examples are the only
documentation...).

For short description of the single scripts, see [Example
Scripts](doc/example_scripts.md).


Calibrate / Initialize the Leg Position
=======================================

Since the motor encoders only provide relative information, the absolute
position of the leg at start is unknown. For the demos, a simple initialization
step is implemented, where the user is asked to manually move the joints of the
leg in positive direction until the limit stop. Based on this, the zero position
(where the leg is fully stretched) is determined.

For this to work, the offset between limit stop and zero position has to be
known. To determine it, the script `leg_calibrate.py` is used. Simply run the
script and follow its instructions. The result is written to the file
"calibration_data.p" which is then loaded by the other scripts.

A demo of how the initialization is done in the code is found in
`leg_init_position.py`. This script can also be used to verify that the
calibration is correct.





