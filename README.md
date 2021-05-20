# motor-control
Motor Control Unit and ROS Node

## Compiling & Flashing the MCU

- If you do not already have the dependencies installed: `make deps`
- `make flash`

You can also run the MCU node independently with `make run`.

## PID tuning

The PID is currently disabled via the constant expressions at the top of `mcu.ino`.
It is recommended that the PID parameters be carefully re-tuned based on the surface you intend to run the car on.

The PID algorithm is easy to implement in software, however there is some additional cost from the time taken to tune the three (P, I, and D) terms for the controller to respond correctly for our particular system. Tuning the parameters for the controller was done on an automated test bed where a control signal was transmitted to the motor control unit with a desired velocity and plots were automatically generated for visual analysis of the measured vs desired response. The tuning methodology used is enumerated below:
1. Proportional gain determines how responsive the controller is to changes in error between the desired and measured output. Higher gain may lead to instability and lower gain reduces steady-state error. The proportional term is increased until the measured response quickly reaches a steady state below the desired value.
2. Integral gain may accelerate the approach toward the desired value. However, a high gain may cause significant overshoot or oscillation without approaching a steady state. The integral term is increased until the response overshoots and then oscillates about the desired value while quickly reaching a steady state.
3. Derivative gain reduces overshoot and oscillation prior to reaching steady state. However, higher gain will also increase the settling time needed to reach the steady state. The derivative term is increased until an acceptable balance is reached between a reduction of overshoot and the increase in rise time and settling time prior to reaching steady-state.

Feel free to use the test scripts in `test/` to check the tuning. It is recommended that you do most of the tuning with the car suspended, then dial in the final parameters with the car running moving on the desired road surface.
