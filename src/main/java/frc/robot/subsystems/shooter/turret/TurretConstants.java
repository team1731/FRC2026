package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.lib.frc1731.PIDGains;
import frc.robot.Ports;

public class TurretConstants {
    public static final double kGearRatio = 1d / (15d / 40d * 15d / 40d * 40d / 160d); // 256.00 : 9.00 overall reduction;
    public static final double kRotorToSensor = 1d / (15d / 40d  * 15d / 40d / 5d);
    public static final double kSensorToMech = 28.44 / 35.555; // 4.00 : 5.00 overall reduction
    public static final double kEpsilon = 1d; // Degrees;

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
        .withAbsoluteSensorDiscontinuityPoint(0.265)
        .withMagnetOffset(-0.09326171875)
        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
    );

    public static final CANcoderConfiguration kRightCANCoderConfigs = new CANcoderConfiguration()
    .withMagnetSensor(new MagnetSensorConfigs()
        .withAbsoluteSensorDiscontinuityPoint(0.742)
        .withMagnetOffset(0.035888671875)
        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
    );
        
    public static final TurretConfiguration kLeftTurretConfigs = new TurretConfiguration(
        "Left",
        Ports.kLeftTurretConfigs,
        Ports.kLeftTurretCANCoderId,
        kLeftFeedbackConfigs,
        kLeftCANCoderConfigs,
        101.0,
        -311,
        kRobotToLeftTurret
    );

    public static final TurretConfiguration kRightTurretConfigs = new TurretConfiguration(
        "Right",
        Ports.kRightTurretConfigs,
        Ports.kRightTurretCANCoderId,
        kRightFeedbackConfigs,
        kRightCANCoderConfigs,
        319.7,
        -92.5,
        kRobotToRightTurret
    );
}