package frc.robot.subsystems.intake;

import frc.lib.frc1731.PIDGains;

public class IntakeConstants {
    public static final PIDGains kPivotGains = new PIDGains()
        .setP(60)
        .setD(.5)
        // .setS(.25)
        .setV(5.76)
        .setA(0.48);

    // Initial VelocityVoltage gains, using motor-shaft RPS. Validate under load.
    public static final PIDGains kRollerGains = new PIDGains()
        .setP(0.05).setI(0.0).setV(0.10);

    // Provisional motor-shaft speeds; not conversions from duty cycle.
    public static final double kRollerIntakeRPS = 110;
    public static final double kRollerEjectRPS = -90;

    public static final double kRollerCurrentLimit = 120;
    public static final double kPivotCurrentLimit = 60.0;

    public static final double kPivotGearRatio = 48.0; // reduction

    // Optional VelocityVoltage + FOC baseline for the X44, not active.
    // Retains the existing 0.10 V/(motor RPS) estimate; this is NOT a measured X44 FOC kV.
    // Characterize kS/kV first; leave I/D/A zero during initial velocity tuning.
    public static final PIDGains kRollerFOCVoltageGains = new PIDGains()
        .setP(0.05).setI(0).setD(0).setS(0).setV(0.10).setA(0);

    // Optional MotionMagicVoltage + FOC baseline for the X60 pivot, not active.
    // Assumes CANcoder feedback in output-shaft rotations and the stated 48:1 reduction.
    // Gentler provisional P; disable uncharacterized acceleration FF for initial tuning.
    // Verify encoder setup/profile constraints and measure gravity/friction compensation.
    public static final PIDGains kPivotFOCVoltageGains = new PIDGains()
        .setP(20).setI(0).setD(0.5).setS(0).setV(0.124 * kPivotGearRatio).setA(0).setG(0);

    public static final double kPivotIntakeRotations = -0.144287;
    public static final double kPivotStowRotations = 0;
    public static final double kPivotEpsilon = 0.01;
}
