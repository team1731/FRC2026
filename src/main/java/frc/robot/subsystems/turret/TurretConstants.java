package frc.robot.subsystems.turret;

import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.PortConfig;

public class TurretConstants {
    public static final double kGearRatio = 20d / 160d;
    public static final double kEpsilon = 1d; // Degrees;

    public static final PIDGains kPositionGains = new PIDGains().setP(1d);
    public static final PortConfig kLeftPortConfigs = new PortConfig("rio", 30, false);
}