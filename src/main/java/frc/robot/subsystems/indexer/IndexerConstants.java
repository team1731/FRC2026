package frc.robot.subsystems.indexer;

import frc.lib.frc1731.PIDGains;

public class IndexerConstants {
    public static final PIDGains kPIDGains = new PIDGains()
        .setP(0.10).setI(0).setD(0).setS(0.15).setV(0.12).setA(0);

    public static final double kCurrentLimit = 60.0;

    // Provisional motor-shaft RPS for VelocityVoltage; validate under load.
    public static final double kFeedRPS = 50;
    public static final double kEjectRPS = -90;
}
