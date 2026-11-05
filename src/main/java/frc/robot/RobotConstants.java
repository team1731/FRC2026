package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Robot-wide constants that affect startup, logging, controls, CAN buses, and mode selection.
 */
public final class RobotConstants {
    /** Primary CANivore bus for drivetrain and default mechanism devices. */
    public static final CANBus kMainCANBus = new CANBus("Left CANivore");

    /** Secondary CANivore bus available for additional mechanisms. */
    public static final CANBus kSecondCANBus = new CANBus("Right CANivore");

    /** Competition event key (e.g. 2026vabla). Empty uses the FMS event name. */
    public static final String kLogEventKey = "";

    /** Publish one AdvantageKit NT4 stream for AdvantageScope live viewing. */
    public static final boolean kPublishLogsToNT = true;

    /** Save complete local logs independently of the live dashboard connection. */
    public static final boolean kLogToWPILog = true;

    /** High-rate Phoenix logs are opt-in for characterization and CAN investigations. */
    public static final boolean kLogPhoenixSignals = false;

    /** Human-readable robot name used by visualization and logging code. */
    public static final String kRobotName = "Raptor";

    /** Suffix used by autos or deploy files that intentionally do not use VSLAM. */
    public static final String kNoVSLAMPostfix = "_NoVSLAM";

    /** Default PathPlanner auto name, matching the deployed filename without .auto. */
    public static final String kAutoDefault = "Comp_RightOverBumpX2";

    /** SmartDashboard key for the autonomous chooser. */
    public static final String kAutoCodeKey = "Auto Selector";

    /** USB port for the driver controller. */
    public static final int kDriverControllerPort = 0;

    /** USB port for the operator controller. */
    public static final int kOperatorControllerPort = 1;

    /** roboRIO brownout threshold in volts. */
    public static final double kBrownoutVoltage = 6.50;

    private static final RobotType robot = RobotType.COMPBOT;

    /** Enables extra tuning-only behavior and dashboards when supported by subsystems. */
    public static final boolean tuningMode = false;

    /** NetworkTables keys used by mechanism visualization utilities. */
    public static class VizConstants {
        /** NetworkTables table used for mechanism visualization state. */
        public static final String kTableKey = "MechViz";

        /** Shared table handle for visualization publishers. */
        public static final NetworkTable kVisualizerTable =
                NetworkTableInstance.getDefault().getTable(kTableKey);

        /** Entry used to toggle or inspect visualization debug behavior. */
        public static final String kDebugEntry = kRobotName + "/Debug";
    }

    /**
     * Current mode of the codebase (Real robot, simulation, log replay)
     */
    public static Mode getMode() {
        if (RobotBase.isReal()) return Mode.REAL;
        String replayPath = System.getenv("AKIT_LOG_PATH");
        return replayPath != null && !replayPath.isBlank() ? Mode.REPLAY : Mode.SIM;
    }

    /**
     * Which robot is actively being used. Useful for using multiple robots within same codebase (practice bot, alpha bot, etc.)
     */
    public static RobotType getRobot() {
        return Robot.isSimulation() ? RobotType.SIMBOT : robot;
    }

    /** Runtime environment selected for logging, simulation, and hardware behavior. */
    public enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    /** Robot identity used to choose real hardware behavior versus pure simulation. */
    public enum RobotType {
        /** Competition robot configuration. */
        COMPBOT,

        /** Simulation-only robot configuration. */
        SIMBOT
    }
}
