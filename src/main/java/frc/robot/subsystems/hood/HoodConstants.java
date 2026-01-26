package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import frc.lib.frc1678.sim.PivotSim.PivotSimConstants;
import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.PortConfig;

public class HoodConstants {
    public static final double kGearRatio = (15d / 64d * 32d / 64d * 20d / 380d); // 162.13 : 1.00 overall reduction
    public static final double kStartDegrees = 0.0; // Degrees
    public static final double kMaxDegrees = 180.0; // Degrees
    public static final Distance kHoodRadius = Inches.of(6d); // Radius of the hood
    public static final Mass kHoodMass = Pounds.of(2d); // Weight of the moving hood

    public static final PortConfig kLeftHoodConfig = new PortConfig(0);
    public static final PortConfig kRightHoodConfig = new PortConfig(0);

    public static final PIDGains kPositionGains = new PIDGains()
    .setP(0.0)
    .setD(0.0)
    .setV(0.0)
    .setS(0.0)
    .setA(0.0)
    ;

    public static final PivotSimConstants kSimConstants = new PivotSimConstants()
        .withConstraints(kStartDegrees, kMaxDegrees, kStartDegrees, kHoodRadius.in(Meters))
        .withPhysics(kGearRatio, 0.5 * kHoodMass.in(Kilograms) * Math.pow(kHoodRadius.in(Meters), 2), false)
        .withMotor(DCMotor.getKrakenX60(1));
}