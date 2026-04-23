package frc.robot.subsystems.squeezer;

import frc.lib.frc1731.PIDGains;

public class SqueezerConstants {
    public static final double kGearRatio = 25.0 / 1.0; // reduction
    public static final double kMaxPosition = 0.5; // rotations

    public static final double kCurrentLimit = 30.0; // amps
    public static final double kEpsilon = 0.1; // rotations

    public static final PIDGains kPositionGains = new PIDGains()
        .setP(0.5)
        .setV(0.12);
}