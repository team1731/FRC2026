package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.subsystem.converter.AngularSubsystemConverter;
import frc.robot.Constants;
import frc.robot.Ports;

public class TurretConstants {
    public static final double kGearRatio = (15d / 40d * 15d / 40d * 40d / 160d); // 256.00 : 9.00 overall reduction;
    public static final double kRotorToSensor = 1d / (15d / 40d  * 15d / 40d / 5d);
    public static final double kSensorToMech = 1d;
    public static final double kEpsilon = 1d; // Degrees;

    public static final Transform3d kLeftTurretToRobot = new Transform3d(
        Units.inchesToMeters(-7.25d), 
        Units.inchesToMeters(-5.75d), 
        Units.inchesToMeters(11.4125d),
        new Rotation3d()
    );

    public static final Transform3d kRightTurretToRobot = new Transform3d(
        Units.inchesToMeters(7.25d), 
        Units.inchesToMeters(-5.75d), 
        Units.inchesToMeters(11.4125d),
        new Rotation3d()
    );

    public static final AngularSubsystemConverter kConverter = new AngularSubsystemConverter(kGearRatio);

    public static final double[] kLeftRotationsRange = new double[] {-0.2453, 0.3806};
    public static final double[] kRightRotationsRange = new double[] {-0.7321, 0.1616};

    public static final double[] kLeftDegreesRange = new double[] {-103, 172};
    public static final double[] kRightDegreesRange = new double[] {-128, 266};

    public static final PIDGains kPositionGains = new PIDGains()
        .setP(30)
        .setD(0.01)
        .setV(0.12)
        .setS(0.25)
        .setA(0.01);

    public static final MotionMagicConfigs kMotionMagicConfigs = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(1)
        .withMotionMagicAcceleration(1);

    public static final FeedbackConfigs kLeftFeedbackConfigs = new FeedbackConfigs()
        .withFusedCANcoder(new CoreCANcoder(Ports.kLeftTurretCANCoderId, Constants.kLeftCANBus))
        .withRotorToSensorRatio(kRotorToSensor)
        .withSensorToMechanismRatio(kSensorToMech);

    public static final FeedbackConfigs kRightFeedbackConfigs = new FeedbackConfigs()
        .withFusedCANcoder(new CoreCANcoder(Ports.kRightTurretCANCoderId, Constants.kRightCANBus))
        .withRotorToSensorRatio(kRotorToSensor)
        .withSensorToMechanismRatio(kSensorToMech);

    public static final double kRightMinDegrees = -128;
    public static final double kRightMaxDegrees = 266;

    public static final double kCurrentLimit = 40d; // Amps

    public static final DCMotor kDCMotor = DCMotor.getKrakenX60(1);

    public static final Mass kTurretMass = Pounds.of(6d);
    public static final Distance kTurretRadius = Inches.of(8d);
    public static final Angle kMinAngle = Degrees.of(-45);
    public static final Angle kMaxAngle = Degrees.of(225d);
}