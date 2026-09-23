package frc.robot.subsystems.kicker;

import frc.lib.frc1731.PIDGains;

public class KickerConstants {
    // Motor rotations per roller rotation (less than one is a speed increase).
    public static final double kGearRatio = 32.0 / 60.0;

    public static final PIDGains kPIDGains = new PIDGains()
        .setP(0.15).setI(0).setD(0).setS(0.15).setV(0.12).setA(0);

    public static final double kCurrentLimit = 60.0;

    public static final double kRollerDiameter = 1.398; // inches

    // Provisional motor-shaft RPS for VelocityVoltage; validate under load.
    public static final double kFeedRPS = 65;
    public static final double kEjectRPS = -50;
}
