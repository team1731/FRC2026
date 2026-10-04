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
