package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.lib.frc1731.PIDGains;
import frc.robot.Ports;

public class TurretConstants {
    public static final double kGearRatio = 1d / (15d / 40d * 19d / 36d * 36d / 164d); // 256.00 : 9.00 overall reduction;
    public static final double kRotorToSensor = 1d / (15d / 40d  * 19d / 36d / 5d);
    public static final double kSensorToMech = 1.0 / (kRotorToSensor / kGearRatio); // 4.00 : 5.00 overall reduction
    public static final double kEpsilon = 3d; // Degrees;

    public static final Translation3d kRobotToLeftTurret = new Translation3d(
        Units.inchesToMeters(-5.75d),
        Units.inchesToMeters(7.25d), 
        Units.inchesToMeters(11.4125d)
    );

    public static final Translation3d kRobotToRightTurret = new Translation3d(
        Units.inchesToMeters(-5.75d),
        Units.inchesToMeters(-7.25d),
        Units.inchesToMeters(11.4125d)
    );

    public static final PIDGains kPositionGains = new PIDGains()
        .setP(60)
        .setD(0.5)
        .setS(0.2)
        .setV(0.12*kGearRatio)
        .setA(0.01)
    ;

    public static final double kMaxTurretVelocity = 4.0; // Rotations/sec
    public static final double kMaxTurretAcceleration = 4.0; // Rotations/sec^2

    public static final double kCurrentLimit = 30d; // Amps

    public static final FeedbackConfigs kLeftFeedbackConfigs = new FeedbackConfigs()
    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
    .withFeedbackRemoteSensorID(29)
    .withRotorToSensorRatio(kRotorToSensor)
    .withSensorToMechanismRatio(kSensorToMech);

    public static final FeedbackConfigs kRightFeedbackConfigs = new FeedbackConfigs()
    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
    .withFeedbackRemoteSensorID(30)
    .withRotorToSensorRatio(kRotorToSensor)
    .withSensorToMechanismRatio(kSensorToMech);

    public static final CANcoderConfiguration kLeftCANCoderConfigs = new CANcoderConfiguration()
    .withMagnetSensor(new MagnetSensorConfigs()
        .withAbsoluteSensorDiscontinuityPoint(0.29)
        .withMagnetOffset(0.009521484375)
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
    );

    public static final CANcoderConfiguration kRightCANCoderConfigs = new CANcoderConfiguration()
    .withMagnetSensor(new MagnetSensorConfigs()
        .withAbsoluteSensorDiscontinuityPoint(0.7325)
        .withMagnetOffset(-0.26171875)
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
    );
        
    public static final TurretConfiguration kLeftTurretConfigs = new TurretConfiguration(
        "Left",
        Ports.kLeftTurretConfigs,
        Ports.kLeftTurretCANCoderId,
        kLeftFeedbackConfigs,
        kLeftCANCoderConfigs,
        125, // 0.257
        -261, // -0.662
        kRobotToLeftTurret
    );

    public static final TurretConfiguration kRightTurretConfigs = new TurretConfiguration(
        "Right",
        Ports.kRightTurretConfigs,
        Ports.kRightTurretCANCoderId,
        kRightFeedbackConfigs,
        kRightCANCoderConfigs,
        291, // 0.730
        -105, // -0.265
        kRobotToRightTurret
    );
}