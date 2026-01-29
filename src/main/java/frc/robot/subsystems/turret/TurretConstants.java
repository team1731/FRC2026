package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.PortConfig;
import frc.lib.frc1731.subsystem.converter.AngularSubsystemConverter;

public class TurretConstants {
    public static final double kGearRatio = 20d / 160d;
    public static final double kEpsilon = 1d; // Degrees;

    public static final AngularSubsystemConverter kConverter = new AngularSubsystemConverter(kGearRatio);

    public static final PIDGains kPositionGains = new PIDGains().setP(0d).setD(0.0).setS(0.01).setV(5);
    public static final PortConfig kLeftPortConfigs = new PortConfig("rio", 30, false);

    public static final double kCurrentLimit = 40d; // Amps

    public static final DCMotor kDCMotor = DCMotor.getKrakenX60(1);

    public static final Mass kTurretMass = Pounds.of(6d);
    public static final Distance kTurretRadius = Inches.of(8d);
    public static final Angle kMinAngle = Degrees.of(-45);
    public static final Angle kMaxAngle = Degrees.of(225d);
}