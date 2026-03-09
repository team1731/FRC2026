package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.PortConfig;
import frc.lib.frc1731.subsystem.converter.AngularSubsystemConverter;
import frc.robot.subsystems.shooter.turret.TurretSubsystem.TurretConfigs;

public class TurretConstants {
    public static final double kGearRatio = (15d / 40d * 15d / 40d * 40d / 160d); // 256.00 : 9.00 overall reduction;
    public static final double kRotorToSensor = 1d / (15d / 40d  * 15d / 40d / 5d);
    public static final double kSensorToMech = 1d;
    public static final double kEpsilon = 1d; // Degrees;

    public static final Transform3d kLeftTurretToRobot = new Transform3d(
        Units.inchesToMeters(-5.75d), 
        Units.inchesToMeters(7.25d), 
        Units.inchesToMeters(11.4125d),
        new Rotation3d()
    );

    public static final Transform3d kRightTurretToRobot = new Transform3d(
        Units.inchesToMeters(-5.75d), 
        Units.inchesToMeters(-7.25d), 
        Units.inchesToMeters(11.4125d),
        new Rotation3d()
    );

    public static final AngularSubsystemConverter kConverter = new AngularSubsystemConverter(kGearRatio);

    public static final PIDGains kPositionGains = new PIDGains()
        .setP(60)
        .setD(0.5)
        .setS(0.2)
        .setA(0.01)
    ;

    public static final PortConfig kLeftPortConfigs = new PortConfig("Right CANivore", 22, false);
    public static final PortConfig kRightPortConfigs = new PortConfig("Left CANivore", 18, false);

    public static final double kCurrentLimit = 30d; // Amps

    public static final FeedbackConfigs kLeftFeedbackConfigs = new FeedbackConfigs()
    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
    .withFeedbackRemoteSensorID(29)
    .withRotorToSensorRatio(35.555)
    .withSensorToMechanismRatio(28.44 / 35.555);

    public static final FeedbackConfigs kRightFeedbackConfigs = new FeedbackConfigs()
    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
    .withFeedbackRemoteSensorID(30)
    .withRotorToSensorRatio(35.555)
    .withSensorToMechanismRatio(28.44 / 35.555);

    public static final CANcoderConfiguration kLeftCANCoderConfigs = new CANcoderConfiguration()
    .withMagnetSensor(new MagnetSensorConfigs()
        .withAbsoluteSensorDiscontinuityPoint(0.282)
        .withMagnetOffset(0.016357421875)
        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
    );

    public static final CANcoderConfiguration kRightCANCoderConfigs = new CANcoderConfiguration()
    .withMagnetSensor(new MagnetSensorConfigs()
        .withAbsoluteSensorDiscontinuityPoint(0.742)
        .withMagnetOffset(-0.283447265625)
        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
    );
        
    public static final TurretConfigs kLeftTurretConfigs = new TurretConfigs(
        "Left",
        kLeftPortConfigs,
        29,
        kLeftFeedbackConfigs,
        kLeftCANCoderConfigs,
        93.0,
        -303.92,
        kLeftTurretToRobot
    );

    public static final TurretConfigs kRightTurretConfigs = new TurretConfigs(
        "Right",
        kRightPortConfigs,
        30,
        kRightFeedbackConfigs,
        kRightCANCoderConfigs,
        319.7,
        -92.5,
        kLeftTurretToRobot
    );
}