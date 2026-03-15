package frc.robot.subsystems.shooter.turret;

import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

import java.util.function.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.frc1731.Utils;
import frc.lib.frc1731.field.FieldPositions;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.robot.subsystems.BaseSubsystem;

public class TurretSubsystem extends BaseSubsystem {
    private MotorIOTalonFX motor;
    private CANcoder cancoder;
    private Translation2d robotToTurret;
    private Supplier<Pose2d> swervePoseSupplier;

    private double targetDegrees = 0;
    private double minDegrees, maxDegrees;

    private Translation2d targetTranslation;

    public TurretSubsystem(TurretConfiguration config, Supplier<Pose2d> swervePoseSupplier, boolean enabled) {
        super(config.name(), config, enabled);
        this.swervePoseSupplier = swervePoseSupplier;
    }

    @Override
    protected void initializeHardware() {
        TurretConfiguration turretConfig = (TurretConfiguration)config.get();
        motor = new MotorIOTalonFX(turretConfig.motorConfigs());

        cancoder = new CANcoder(turretConfig.cancoderID());

        CANcoderConfiguration coderConfig = new CANcoderConfiguration();
        coderConfig.MagnetSensor.MagnetOffset = turretConfig.cancoderConfigs().MagnetSensor.MagnetOffset;
        coderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = turretConfig.cancoderConfigs().MagnetSensor.AbsoluteSensorDiscontinuityPoint;
        coderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
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

        motorConfig.Slot0.kP = 60.0; 
        motorConfig.Slot0.kS = 0.2;
        motorConfig.Slot0.kA = 0.01;
        motorConfig.Slot0.kI = 0;
        motorConfig.Slot0.kD = 0.5;
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = 4;
        motorConfig.MotionMagic.MotionMagicAcceleration = 4;

        motor.getMotor().getConfigurator().apply(motorConfig);
        motor.getMotor().setPosition(cancoder.getAbsolutePosition().waitForUpdate(0.2).getValueAsDouble());

        this.robotToTurret = turretConfig.robotToTurret().toTranslation2d();
        this.minDegrees = turretConfig.minDegrees();
        this.maxDegrees = turretConfig.maxDegrees();
    }

    public Pose2d getTurretPose() {
        if (!isEnabled()) return new Pose2d();
        Translation2d swerve = swervePoseSupplier.get().getTranslation();
        return new Pose2d(swerve.plus(robotToTurret.rotateBy(swervePoseSupplier.get().getRotation())), swervePoseSupplier.get().getRotation());
    }

    public boolean atTarget() {
        if (!isEnabled()) return true;
        return Utils.isWithin(motor.getRotations() * 360.0, targetDegrees, kEpsilon);
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Pose", getTurretPose());
        logger.log("Current Degrees", motor.getRotations() * 360.0);
        logger.log("Target Degrees", targetDegrees);
        logger.log("At Target", atTarget());
        logger.log("Min Degrees", minDegrees);
        logger.log("Max Degrees", maxDegrees);

        if (targetTranslation != null) logger.log("TargetDistance", targetTranslation.minus(getTurretPose().getTranslation()).getNorm());
    }

    public Command trackHub() {
        return track(FieldPositions.kHub.get());
    }

    public Command track(Supplier<Translation2d> target) {
        return this.setDegrees(() -> {
            this.targetTranslation = target.get();
            Translation2d turretToTarget = target.get().minus(getTurretPose().getTranslation());
            return Math.atan(turretToTarget.getY() / turretToTarget.getX()) * 180 / Math.PI;
        });
    }

    public Command track(Translation2d target) {
        return this.track(() -> target);
    }

    public Command setDegrees(DoubleSupplier degrees) {
        return run(() -> {
            targetDegrees = degrees.getAsDouble() - (swervePoseSupplier.get().getRotation().getDegrees() + 180 % 360);
            if (targetDegrees <= minDegrees) targetDegrees += 360;
            if (targetDegrees >= maxDegrees) targetDegrees -= 360;
            motor.setPosition(targetDegrees / 360.0);
        }).withName("SetDegrees");
    }

    public Command setDegrees(double degrees) {
        return this.setDegrees(() -> degrees);
    }
}