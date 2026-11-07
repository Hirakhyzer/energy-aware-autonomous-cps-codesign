# Path toward hardware and hardware-in-the-loop validation

1. Select a low-power embedded controller with documented CPU frequency scaling, a measured current monitor, a localization sensor, and a mobile robot platform with safe independent motor cut-off.
2. Log hardware timestamps at task entry/exit, sensor sampling, PWM changes, battery voltage/current, and waypoint arrival. Keep clocks synchronized.
3. Fit separate standby/active CPU power, sensor read/wake energy, communication energy, motor power vs speed and turn, and switching overhead from repeated trials.
4. Calibrate timing and energy model parameters using training data; reserve new routes and loads for validation only.
5. Compare predicted vs measured joules, deadline misses, energy per completed mission, tracking error, battery state-of-charge, and variance across repetitions.
6. Separate the main runtime energy measurement from any external measurement electronics; explicitly account for optimizer computation and operating-mode transition costs.
7. Implement independent emergency stop and collision avoidance, validate hard actuator limits, and complete a safety review **before** allowing autonomous hardware motion.

No current code in this repository actuates a real robot, changes CPU operating modes on a real board, or provides a deployment-level safety shield.
