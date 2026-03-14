package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.PortConfig;
import frc.lib.frc1731.subsystem.converter.AngularSubsystemConverter;

public class HoodConstants {
    public static final double kGearRatio = (15d / 64d * 32d / 64d * 20d / 380d); // 162.13 : 1.00 overall reduction

    public static final AngularSubsystemConverter kConverter = new AngularSubsystemConverter(kGearRatio);

    public static final Angle kEpsilon = Degrees.of(1d);

    // public static final Angle kStartAngle = Degrees.of(15.5);
    // public static final Angle kMaxAngle = Degrees.of(30.0);
    // public static final Angle kAngleRange = kMaxAngle.minus(kStartAngle);

    // public static final Angle kStartRotations = kConverter.toMotor(kStartAngle);
    // public static final Angle kMaxRotations = kConverter.toMotor(kMaxAngle);
    public static final Angle kStartRotations = Rotations.of(0);
    public static final Angle kMaxRotations = Rotations.of(7.25);
    public static final Angle kRotationsRange = kMaxRotations.minus(kStartRotations);
    public static final double kMaxRotationsValue = -7d;

    public static final Distance kHoodRadius = Inches.of(8.4d); // Radius of the hood
    public static final Mass kHoodMass = Pounds.of(2d); // Weight of the moving hood

    public static final PortConfig kLeftHoodConfig = new PortConfig("Right CANivore", 23, true);
    public static final PortConfig kRightHoodConfig = new PortConfig("Left CANivore", 19, false);

    public static final double kCurrentLimit = 40d; // Amps

    public static final PIDGains kPositionGains = new PIDGains()
        .setP(4.8)
        .setD(0.1)
        .setV(0.12)
        .setS(0.25)
        .setA(0.01)
    ;

    // public static final PIDGains kSimGains = new PIDGains().setP(1).setD(0);

    public static final DCMotor kDCMotor = DCMotor.getMinion(1); 
    // public static final PivotSimConstants kSimConstants = new PivotSimConstants()
    //     .withConstraints(kStartAngle.in(Degrees), kMaxAngle.in(Degrees), kStartAngle.in(Degrees), kHoodRadius.in(Meters))
    //     .withPhysics(kGearRatio, 0.5 * kHoodMass.in(Kilograms) * Math.pow(kHoodRadius.in(Meters), 2), false)
    //     .withMotor(kDCMotor);
}