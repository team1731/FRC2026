# FRC2026

## Elastic driver dashboard

The deployed `src/main/deploy/elastic-layout.json` contains the autonomous selector
and Oculus `isConnected` / `isTracking` indicators. The robot serves it on port
5800 in both real and simulated runs.

One-time setup in Elastic:

1. Connect to the robot (team 1731), or start **WPILib: Simulate Robot Code** and
   set Elastic's IP address mode to **Localhost** for simulation (NT4 port 5810).
2. Choose **File > Load Layout From Robot** (`Ctrl+D`), select `elastic-layout`,
   and choose **Overwrite** to install the Driver Dashboard tab.
   Alternatively, open `src/main/deploy/elastic-layout.json` locally in Elastic.
3. Elastic saves the layout and restores these widgets automatically on future
   launches. When switching back to the real robot, restore the Driver Station
   or team-number connection mode.

The autonomous selector uses the existing `/SmartDashboard/Auto Selector`
chooser. Oculus status is published under `/SmartDashboard/Oculus/` and recorded
in AdvantageKit under `Oculus/`. Without a connected headset, including normal
simulation runs, both indicators are false; they are not simulated tracking.

The layout is also available at `http://localhost:5800/elastic-layout.json`
while simulation is running. Robot code makes the layout available for download;
the initial import is an Elastic setting, not a robot-controlled operation.

## Power monitoring

`Robot.power` owns the power-distribution device and configures AdvantageKit before
`Logger.start()`. The default constructor detects the default CTRE PDP (CAN 0)
or REV PDH (CAN 1). For another ID, construct it with an explicit
`new PowerDistribution(canId, ModuleType.kRev)` or `ModuleType.kCTRE`.
Real operation logs measured distribution current; `Robot.power.getTotalCurrent()`
returns amps. Team telemetry also records voltage, watts, brownout, and CAN bus-off count.

Simulation updates after mechanism periodic updates. Virtual channels 0 through 7
represent IntakeDeploy, IntakeRoller, Indexer, Kicker, Hood, Flywheel, Hopper, and
Swerve, respectively; these are mechanism groups, not physical breaker channels.
The flywheel model includes four motors and the kicker model includes two; each
model is counted once. Mechanism current is converted to estimated supply current
using absolute duty cycle. Swerve uses CTRE's simulated supply current.
Negative regeneration and invalid readings are excluded. BatterySim supplies the
same loaded voltage to PDPSim and RoboRioSim.

Named estimates are logged under `PowerSubsystem/SimLoads` in SmartLogs; the
AdvantageKit `PowerDistribution` table contains the simulated channel values and
total. Use `power.addSimLoad("Electronics", () -> estimatedAmps)` for additional
loads. IntakeDeploy currently has no attached mechanism model, so its load falls
back to vendor simulation rather than a mechanism physics estimate. Simulation
accuracy depends on model parameters and which loads have been registered.
