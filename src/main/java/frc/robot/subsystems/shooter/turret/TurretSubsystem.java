package frc.robot.subsystems.shooter.turret;

import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

import java.util.function.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.frc1731.Utils;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.robot.Robot;
import frc.robot.subsystems.BaseSubsystem;

public class TurretSubsystem extends BaseSubsystem {
    private MotorIOTalonFX motor;
    private CANcoder cancoder;
    private Translation2d robotToTurret;
    private Supplier<Pose2d> swervePoseSupplier;
    private TurretIOInputs inputs = new TurretIOInputs();

    public TurretSubsystem(TurretConfiguration config, Supplier<Pose2d> swervePoseSupplier, boolean enabled) {
        super(config.name(), config, enabled);
        this.swervePoseSupplier = swervePoseSupplier;
        inputs.minDegrees = config.minDegrees();
        inputs.maxDegrees = config.maxDegrees();
    }

    @Override
    protected void initializeHardware() {
        TurretConfiguration turretConfig = (TurretConfiguration)config.get();
        motor = new MotorIOTalonFX(turretConfig.motorConfigs());
        cancoder = new CANcoder(turretConfig.cancoderID(), new CANBus(turretConfig.motorConfigs().kBus));

        CANcoderConfiguration coderConfig = new CANcoderConfiguration();
        coderConfig.MagnetSensor.MagnetOffset = turretConfig.cancoderConfigs().MagnetSensor.MagnetOffset;
        coderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = turretConfig.cancoderConfigs().MagnetSensor.AbsoluteSensorDiscontinuityPoint;
        coderConfig.MagnetSensor.SensorDirection = turretConfig.cancoderConfigs().MagnetSensor.SensorDirection;
        cancoder.getConfigurator().apply(coderConfig);

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        motorConfig.Feedback.FeedbackRemoteSensorID = cancoder.getDeviceID();
        motorConfig.Feedback.RotorToSensorRatio = turretConfig.feedbackConfigs().RotorToSensorRatio;
        motorConfig.Feedback.SensorToMechanismRatio = turretConfig.feedbackConfigs().SensorToMechanismRatio;

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.MotorOutput.Inverted = turretConfig.motorConfigs().kInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = turretConfig.maxDegrees() / 360.0;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = turretConfig.minDegrees() / 360.0;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        motorConfig.CurrentLimits.StatorCurrentLimit = 30d;

        motorConfig.Slot0.kP = kPositionGains.kP; 
        motorConfig.Slot0.kS = kPositionGains.kS;
        motorConfig.Slot0.kA = kPositionGains.kA;
        motorConfig.Slot0.kI = kPositionGains.kI;
        motorConfig.Slot0.kD = kPositionGains.kD;
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = kMaxTurretVelocity;
        motorConfig.MotionMagic.MotionMagicAcceleration = kMaxTurretAcceleration;

        // motorConfig.Slot0.kP = 60.0; 
        // motorConfig.Slot0.kS = 0.2;
        // motorConfig.Slot0.kA = 0.01;
        // motorConfig.Slot0.kI = 0;
        // motorConfig.Slot0.kD = 0.5;
        // motorConfig.MotionMagic.MotionMagicCruiseVelocity = 4;
        // motorConfig.MotionMagic.MotionMagicAcceleration = 4;

        motor.getMotor().getConfigurator().apply(motorConfig);
        motor.getMotor().setPosition(cancoder.getAbsolutePosition().waitForUpdate(0.2).getValueAsDouble());

        this.robotToTurret = turretConfig.robotToTurret().toTranslation2d();
    }

    public double getError() {
        return inputs.targetDegrees - inputs.currentDegrees;
    }

    public Pose2d getTurretPose() {
        if (!isEnabled()) return new Pose2d();
        Translation2d swerve = swervePoseSupplier.get().getTranslation();
        return new Pose2d(swerve.plus(robotToTurret.rotateBy(swervePoseSupplier.get().getRotation())), new Rotation2d());
    }

    public boolean atTarget() {
        if (!isEnabled()) return true;
        return Utils.isWithin(inputs.currentDegrees, inputs.targetDegrees, kEpsilon);
    }

    public boolean atTarget(double epsilon) {
        if (!isEnabled()) return true;
        return Utils.isWithin(inputs.currentDegrees, inputs.targetDegrees, epsilon);
    }

    public double getTarget() {
        if (!isEnabled()) return 0;
        return inputs.targetDegrees;
    }

    public double getDegrees() {
        if (!isEnabled()) return 0;
        return motor.getRotations() * 360;
    }

    private double calculateBestTurretAngle(double robotHeading, double targetHeading, double current) {
        // double desired = (targetHeading - robotHeading) % 360;
        // if (desired > maxDegrees) desired -= 360;
        // if (desired < minDegrees) desired += 360;
        // return Math.max(minDegrees, Math.min(maxDegrees, desired));
        double desired = (targetHeading - robotHeading + 180) % 360;
        if (desired < 0) desired += 360;
        desired -= 180;

        double path1 = desired;
        double path2 = (desired > 0) ? (desired - 360) : (desired + 360);

        boolean reach1 = (path1 >= inputs.minDegrees && path1 <= inputs.maxDegrees);
        boolean reach2 = (path2 >= inputs.minDegrees && path2 <= inputs.maxDegrees);

        if (reach1 && reach2) {
            return (Math.abs(path1 - current) <= Math.abs(path2 - current)) ? path1 : path2;
        } else if (reach1) return path1;
        else if (reach2) return path2;

        return Math.max(inputs.minDegrees, Math.min(inputs.maxDegrees, path1));
    }

    @Override
    public void periodicTelemetry() {
        inputs.currentDegrees = motor.getRotations() * 360.0;
        inputs.turretPose = getTurretPose();
        inputs.atTarget = atTarget();
     //   logger.processInputs(inputs);
    }

    public Command trackHub() {
        return track(() -> {
            return Robot.isRedAlliance() ? new Translation2d(11.915394, 4.034536) : new Translation2d(4.625594, 4.034536);
        });
    }

    public Command track(Supplier<Translation2d> target) {
        return this.run(() -> {
            Pose2d robotPose = swervePoseSupplier.get();
            Rotation2d robotRotation = robotPose.getRotation();
            
            Translation2d turretFieldPos = robotPose.getTranslation().plus(
                robotToTurret.rotateBy(robotRotation)
            );

            double fieldTargetHeading = Math.toDegrees(Math.atan2(
                target.get().getY() - turretFieldPos.getY(),
                target.get().getX() - turretFieldPos.getX()
            ));

            double output = calculateBestTurretAngle(
                robotRotation.getDegrees(), 
                fieldTargetHeading, 
                inputs.currentDegrees
            );

            inputs.targetDegrees = output;
            inputs.target = new Pose2d(target.get(), new Rotation2d());

            motor.setPosition(output / 360.0);
        });
    }

    public Command track(Translation2d target) {
        return this.track(() -> target);
    }

    public Command setDegrees(DoubleSupplier degrees) {
        return run(() -> {
            double output = degrees.getAsDouble();
            if (output >= inputs.maxDegrees) {
                output -= 360;
            } else if (output <= inputs.minDegrees) {
                output += 360;
            }
            inputs.targetDegrees = output;
            motor.setPosition(output / 360.0);
        });
    }

    public Command setDegrees(double degrees) {
        return this.setDegrees(() -> degrees);
    }
}