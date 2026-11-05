# USB logging and AdvantageScope

## USB recording

Insert a writable USB drive before starting robot code. With `RobotConstants.kLogToWPILog = true`, AdvantageKit records `.wpilog` files in `/U/logs`. If the drive is missing or the directory cannot be written, Driver Station shows a warning; live AdvantageScope publishing can still run. Insert the drive and restart robot code to enable recording.

Desktop simulation records into the repository's ignored `logs/` directory. The standard WPILOGWriter generates session filenames and includes date/event/match information when available. It does not split every match into a separate file during one continuous session.

AdvantageKit starts before mechanisms and controls are created. Its runtime and annotation processor use the same vendordep version. Phoenix signal logging is off by default and can be enabled with `kLogPhoenixSignals` for characterization. DataLogManager is not started separately.

## AdvantageScope live connection

1. Set `RobotConstants.kPublishLogsToNT = true` (the default).
2. Start robot code. In AdvantageScope select **File > Connect to Robot > NetworkTables 4 (AdvantageKit)**. Set the robot address to `10.17.31.2`.
3. For desktop simulation, use **File > Connect to Simulator > NetworkTables 4 (AdvantageKit)** at `127.0.0.1`.
4. Use **Low Bandwidth** live mode over the robot radio.

For recorded data, retrieve the `.wpilog` from USB and open it in AdvantageScope. Viewing a file does not require running robot-code replay. The existing replay path is not yet a validated deterministic replay of all mechanism/vision IO.

## Fields to use

These are file-log paths; live NetworkTables topics add `/AdvantageKit` before them.

| View | Fields |
| --- | --- |
| 2D field | `/RealOutputs/Swerve/Pose` |
| Swerve | `/RealOutputs/Swerve/ModuleStates` and `ModuleTargets` (FL, FR, BL, BR) |
| Mechanisms | `/Subsystems/<SubsystemName>/...`, including current position/velocity, setpoints, and readiness already logged by each subsystem |
| Commands and team outputs | `/RealOutputs/SmartLogs/<SubsystemName>/...` |
| Vision | `/RealOutputs/SmartLogs/Vision/Oculus Connected`, `Oculus Tracking`, `Oculus Seeded`, `Oculus Battery` |
| Build information and console | AdvantageScope Metadata and Console tabs |

Inputs now live under `/Subsystems`; team outputs use `/RealOutputs/SmartLogs` without duplicate slashes. Update old saved field selections. Re-import `src/main/deploy/rebuild-layout.json` into Elastic to pick up the corrected topics.

## Robot check

Record a short disabled/enabled session with USB inserted, retrieve the resulting file, and confirm mechanism signals and swerve fields appear in AdvantageScope. Test startup without USB and confirm the warning appears while robot operation and live viewing continue. Real USB recording still needs this hardware check; compilation alone cannot verify it.

Reference: [AdvantageScope live sources](https://docs.advantagescope.org/overview/live-sources/).
