package frc.robot.subsystems.kicker;

import frc.lib.frc1731.PIDGains;

public class KickerConstants {
    public static final PIDGains kPIDGains = new PIDGains()
        .setP(1.0)
        .setV(0.12)
        .setS(0.15)
        ;

    public static final double kCurrentLimit = 60.0;
}