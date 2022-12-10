# Arduino Files for HSA Robot


## File Descriptions

- `servo_test`: Moves servos to desired angles. Often used to reset a leg.
- `servo_breakin`: Extends and contracts a leg 10 times, ideally breaking in a new leg to achieve consistent behavior. Not always required if assembly / leg cleaning is well-done.
- `serial_test`: Tests serial communication with Python.
- `pmtg`: Listens for servo commands from computer (when running a PMTG policy), then executes them on the servos.
- `libraries`: Contains all the required libraries for all the Arduino files in this project.
- `circular_gait_test`: Runs an open-loop circular gait - defined by a lookup table utility Python file.
- `9dof_imu_test`: Runs and tests a 9 DoF IMU. Deprecated.

The folder `arduino_sketches` contains:
- `inplane_sweep`: Moves an HSA leg in a planar manner and prints out the motor commands. Used with `generate_lookup_table.py`.
- `pmtg_test`: Deprecated
- `stone_gait_9_29`: Runs online reinforcement learning, meant to be used in conjunction with `pipeline_test.py`. Not the current approach where results are found.