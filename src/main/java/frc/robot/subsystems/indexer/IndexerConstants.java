package frc.robot.subsystems.indexer;

import frc.lib.frc1731.PIDGains;

public class IndexerConstants {
    public static final double kGearRatio = 3.0;
    
    public static final PIDGains kPIDGains = new PIDGains()
        .setP(0.10).setI(0).setD(0).setS(0.15).setV(0.12).setA(0);

    // Optional VelocityVoltage + FOC baseline, not active; P/S need physical tuning.
    // X60 estimate kV ~= 12 / (5800 / 60). Feedback is motor RPS, so no gearing scale.
    public static final PIDGains kFOCVoltageGains = new PIDGains()
        .setP(0.10).setI(0).setD(0).setS(0.15).setV(0.124).setA(0);

    public static final double kCurrentLimit = 120.0;
    public static final double kSupplyCurrentLimit = 40.0; // Amps; validate under load.

    public static final double kRollerDiameter = 1.398; // inches

    // Provisional motor-shaft RPS for VelocityVoltage; validate under load.
    public static final double kFeedRPS = 60;
    public static final double kEjectRPS = -90;
}
