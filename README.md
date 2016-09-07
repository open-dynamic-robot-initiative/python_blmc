Python BrushLess Motor Control
==============================

The `blmc` package contains an API for the CAN interface of the motor control
microcontrollers of the Brushless Motor Control Project as well as the
OptoForce sensor.

The scripts in the root directory use this API for several tasks like
evaluating the CAN interface, displaying motor parameters or running simple
velocity/position controllers.

For now, this only works with the *dual_motor_torque_ctrl* code running on the
board.


Requirements
------------

 * python-can
 * numpy
