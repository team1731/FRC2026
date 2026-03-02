package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;

import frc.lib.frc1731.PIDGains;
import frc.robot.Constants;
import frc.robot.Ports;

public class IntakeConstants {
    public static final PIDGains kPivotGains = new PIDGains()
        .setP(20d)
        .setD(0.1)
        .setS(0.5)
        .setV(0.25)
        .setA(0.1);

    public static final PIDGains kPIDGains = new PIDGains().setP(0.12);

    public static final FeedbackConfigs kPivotFeedbackConfigs = new FeedbackConfigs()
        .withRemoteCANcoder(new CoreCANcoder(Ports.kPivotCANcoderId, Constants.kRightCANBus))
        .withRotorToSensorRatio(36d);

    public static final MotionMagicConfigs kMotionMagicConfigs = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(1.5)
        .withMotionMagicAcceleration(2);

    public static final double kPivotCurrentLimit = 40d; // amps
    
    // public static final double kPivotIntakeRotations = 3.5; // rotations
    // public static final double kPivotStowRotations = 0.0; // rotations
    // public static final double kPositionTolerance = 0.2; // rotations

    public static final double kPivotIntakeRotations = -0.091;
    public static final double kPivotStowRotations = 0.038;

    public static final double kEpsilon = 0.001;

    // public static final double kMaxRotations = 3.726;

}
