# 0.19.1 effective-track offline evidence

New physical target mapping, not a hardware speed prediction. Effective395mm used consistently in command allocation and inverse body estimate; physical210mm retained. .40rad/s maps to +/-603.516cps rather than +/-320.857cps. Raw counters, encoder scale, speed estimator, IMU, per-wheel PI and fault/STOP mechanisms unchanged from .19.0. No hardware operations in this step.

Strict/O2/fanalyzer model tests: 1271 mixed-command round trips, saturation preserves curvature, right/left symmetry, straight unchanged, small commands preserved, invalid inputs rejected. Actual runtime/controller/safety with prescribed feedback: both directions and tick wrap hold requested .40rad/s/new wheel targets for15.2s simulated time, then STOP disables outputs. Not a motor/slip model.

Existing simplified controller/trace/safety/encoder regression passed with changed kinematics. Eighty-thousand mixed controller updates and40k NORMAL comparisons apply to the unchanged per-wheel law, not equivalence of body target allocation. Full ARM clean build no warnings/errors; memory, HEX/BIN/vector checks and24 historical artifact hashes preserved.110 production/Core/USB files protected against .19.0 manifest. Details in verification.json and logs/.

Existing ROS395mm is a provisional effective-track assumption supported by approximately3% wheel/IMU agreement in one old-speed capture; it was not fitted in this candidate, and does not prove true body angular speed or repeatability at the new wheel speed. Await user manual flash and a separately observed run. No safety relaxation, encoder count adjustment or enforced minimum angular command.
