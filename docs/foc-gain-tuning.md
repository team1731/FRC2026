# FOC voltage-control tuning baselines

These optional PIDGains objects are experimental starting points, not manufacturer-recommended or characterized mechanism gains. Existing active gains are unchanged.

| Constants class | Optional object | P | I | D | S (V) | V (V/RPS) | A |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| FlywheelConstants | kFOCVoltageGains | 0.20 | 0 | 0 | 0.15 | 0.124 | 0 |
| KickerConstants | kFOCVoltageGains | 0.15 | 0 | 0 | 0.15 | 0.124 | 0 |
| IndexerConstants | kFOCVoltageGains | 0.10 | 0 | 0 | 0.15 | 0.124 | 0 |
| IntakeConstants | kRollerFOCVoltageGains | 0.05 | 0 | 0 | 0 | 0.10 | 0 |
| IntakeConstants | kPivotFOCVoltageGains | 20 | 0 | 0.5 | 0 | 5.952 | 0 |

Velocity rows use motor-shaft RPS. Pivot feedback uses output-shaft rotations: P is V/rotation, D is V/(rotation/s), and V is V/(output rotation/s). Gravity feedforward remains zero and needs mechanism characterization.

The X60 velocity feedforward estimate is 12 / (5800 / 60), rounded to 0.124, from WCP's FOC free-speed specification. Free-speed division is only a first approximation; friction, the separate kS term, and loading require measurement. The X44 intake retains the existing 0.10 estimate rather than using X60 motor data. P/D/S values are provisional, not derived from FOC specifications.

To test a baseline, change that subsystem's `motor.withPIDGains(...)` argument to the corresponding optional object, then deploy. These objects are not live tuning controls. Confirm the Talon FX is Pro licensed and FOC is active before collecting data. Do not use these voltage-unit gains with VelocityTorqueCurrentFOC.

Tune kS and kV at achievable unloaded speeds, then P under load. Leave I at zero initially. Record target/actual RPS, applied voltage, supply/stator current, and battery voltage. Keep current limits fixed while comparing gains. Voltage saturation cannot be corrected by increasing PID gains.

For the pivot, verify encoder bus, offset, feedback units, and motion constraints before applying the baseline. Reduced P does not fix a bad position reference or an infeasible trajectory. Characterize gravity and acceleration separately.

The hood uses Talon FXS and retains its existing kPositionGains; Phoenix FOC is not supported on that controller. The Spark MAX squeezer and chassis heading controllers are not Phoenix FOC loops. Generated swerve motor gains remain managed by TunerConstants.

Sources:
- https://docs.wcproducts.com/welcome/electronics/kraken-x60/kraken-x60-motor/overview-and-features/motor-performance
- https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/closed-loop-requests.html
- https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/talonfx-control-intro.html
