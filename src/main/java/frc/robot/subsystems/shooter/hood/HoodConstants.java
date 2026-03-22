package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.frc1678.sim.PivotSim.PivotSimConstants;
import frc.lib.frc1731.PIDGains;
import frc.robot.Ports;

public class HoodConstants {
    public static final double kGearRatio = 1d / (15d / 64d * 32d / 64d * 20d / 380d); // 162.13 : 1.00 overall reduction

    public static final double kMinRotations = 0;
    public static final double kMaxRotations = 8;
    public static final double kEpsilon = 0.25;

    public static final double kMaxVelocity = 50;
    public static final double kMaxAcceleration = 120;

    public static final HoodConfiguration kLeftHoodConfig = new HoodConfiguration("Left", Ports.kLeftHoodConfig);
    public static final HoodConfiguration kRightHoodConfig = new HoodConfiguration("Right", Ports.kRightHoodConfig);

    public static final double kCurrentLimit = 40d; // Amps

    public static final PIDGains kPositionGains = new PIDGains()
        .setP(4.8)
        .setD(0.1)
        .setV(0.12)
        .setS(0.25)
        .setA(0.01)
    ;

    public static final double kHoodRadiusInches = 8.4d; // Radius of the hood
    public static final double kHoodMassLbs = 2d; // Weight of the moving hood
    public static final PivotSimConstants kSimConstants = new PivotSimConstants()
        .withConstraints(kMinRotations, kMaxRotations * kGearRatio * 360.0, kMinRotations, kHoodRadiusInches)
        .withPhysics(kGearRatio, 0.5 * kHoodMassLbs * Math.pow(kHoodRadiusInches, 2), false)
        .withMotor(DCMotor.getMinion(1));
}