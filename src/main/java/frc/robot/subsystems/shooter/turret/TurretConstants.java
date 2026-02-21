package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.PortConfig;
import frc.lib.frc1731.subsystem.converter.AngularSubsystemConverter;

public class TurretConstants {
    public static final double kGearRatio = (15d / 40d * 15d / 40d * 40d / 160d); // 256.00 : 9.00 overall reduction;
    public static final double kEpsilon = 1d; // Degrees;

    public static final Translation3d kLeftTurretToRobot = new Translation3d(
        Units.inchesToMeters(-7.25d), 
        Units.inchesToMeters(-5.75d), 
        Units.inchesToMeters(11.4125d)
    );

    public static final Translation3d kRightTurretToRobot = new Translation3d(
        Units.inchesToMeters(7.25d), 
        Units.inchesToMeters(-5.75d), 
        Units.inchesToMeters(11.4125d)
    );

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