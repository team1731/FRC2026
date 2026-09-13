package frc.robot.subsystems.kicker;

import frc.lib.frc1731.PIDGains;

public class KickerConstants {
    public static final PIDGains kPIDGains = new PIDGains()
        .setP(0.15).setI(0).setD(0).setS(0.15).setV(0.12).setA(0);

    public static final double kCurrentLimit = 60.0;

    // Provisional motor-shaft RPS for VelocityVoltage; validate under load.
    public static final double kFeedRPS = 50;
    public static final double kEjectRPS = -50;
}
