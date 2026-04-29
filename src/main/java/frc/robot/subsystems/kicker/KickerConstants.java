package frc.robot.subsystems.kicker;

import frc.lib.frc1731.PIDGains;

public class KickerConstants {
    public static final PIDGains kPIDGains = new PIDGains()
        .setP(5
        )
        .setD(0.0)
        .setV(0.0)
        .setS(2.5);
        ;

    public static final double kCurrentLimit = 60.0;
}