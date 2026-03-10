package frc.robot.subsystems.shooter.turret;

import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

import java.util.function.*;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.frc1731.Utils;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.lib.frc6328.FieldConstants;
import frc.robot.subsystems.BaseSubsystem;

public class TurretSubsystem extends BaseSubsystem {
    private MotorIOTalonFX motor;
    private Translation2d robotToTurret;
    private Supplier<Pose2d> swervePoseSupplier;
    private Supplier<ChassisSpeeds> swerveSpeedsSupplier;

    private double targetDegrees = 0;
    private double minDegrees, maxDegrees;

    public TurretSubsystem(TurretConfiguration configs, Supplier<Pose2d> swervePoseSupplier, Supplier<ChassisSpeeds> swerveSpeedsSupplier, boolean enabled) {
        super(configs.name(), enabled);
        if (!isEnabled()) return;
        initializeHardware(configs);

        this.robotToTurret = configs.robotToTurret().getTranslation().toTranslation2d();
        this.swervePoseSupplier = swervePoseSupplier;
        this.swerveSpeedsSupplier = swerveSpeedsSupplier;
        this.minDegrees = configs.minDegrees();
        this.maxDegrees = configs.maxDegrees();
    }

    private void initializeHardware(TurretConfiguration configs) {
        motor = new MotorIOTalonFX(configs.motorConfigs());
        motor.withPIDGains(kPositionGains);
        motor.withCANCoder(configs.feedbackConfigs().FeedbackRemoteSensorID, configs.motorConfigs().kBus, configs.cancoderConfigs());
        motor.withFeedbackConfigs(configs.feedbackConfigs());
        motor.withMotionProfile(kMaxTurretVelocity, kMaxTurretAcceleration);
        motor.withStatorCurrentLimit(kCurrentLimit);
        motor.setSoftLimits(configs.minDegrees() / 360.0, configs.maxDegrees() / 360.0);
        motor.setNeutralMode(NeutralModeValue.Brake);
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
    }

    public Command trackHub() {
        return track(() -> FieldConstants.Hub.topCenterPoint.toTranslation2d());
    }

    public Command track(Supplier<Translation2d> target) {
        return this.setDegrees(() -> {
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