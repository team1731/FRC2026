package frc.robot.subsystems.shooter.turret;

import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

import java.util.function.*;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.frc1731.Utils;
import frc.lib.frc1731.field.FieldPositions;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.robot.subsystems.BaseSubsystem;

public class TurretSubsystem extends BaseSubsystem {
    private MotorIOTalonFX motor;
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
        motor.withPIDGains(kPositionGains);
        motor.withCANCoder(turretConfig.feedbackConfigs().FeedbackRemoteSensorID, turretConfig.motorConfigs().kBus, turretConfig.cancoderConfigs());
        motor.withFeedbackConfigs(turretConfig.feedbackConfigs());
        motor.withMotionProfile(kMaxTurretVelocity, kMaxTurretAcceleration);
        motor.withStatorCurrentLimit(kCurrentLimit);
        motor.setSoftLimits(turretConfig.minDegrees() / 360.0, turretConfig.maxDegrees() / 360.0);
        motor.setNeutralMode(NeutralModeValue.Brake);

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

        if (targetTranslation != null) logger.log("TargetDistance", targetTranslation.minus(getTurretPose().getTranslation()).getNorm());
    }

    public Command trackHub() {
        return track(FieldPositions.kHub.get());
    }

    public Command track(Supplier<Translation2d> target) {
        return this.setDegrees(() -> {
            this.targetTranslation = target.get();
            Translation2d turretToTarget = target.get().minus(getTurretPose().getTranslation());
            return Math.atan(turretToTarget.getY() / turretToTarget.getX());
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