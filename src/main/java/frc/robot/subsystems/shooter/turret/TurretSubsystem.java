package frc.robot.subsystems.shooter.turret;

import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

import java.util.function.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.frc1731.hardware.motor.PortConfig;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.robot.subsystems.BaseSubsystem;

public class TurretSubsystem extends BaseSubsystem {
    private MotorIOTalonFX motor;
    private Transform3d robotToTurret;
    private Supplier<Pose2d> swervePoseSupplier;

    public static record TurretConfigs(String name, PortConfig motorConfigs, int cancoderID, 
                                        FeedbackConfigs feedbackConfigs, CANcoderConfiguration cancoderConfigs,
                                            double forwardLimitDegrees, double reverseLimitDegrees, Transform3d robotToTurret) {}

    public TurretSubsystem(TurretConfigs configs, Supplier<Pose2d> swervePoseSupplier, boolean enabled) {
        super(enabled);
        super.setName(configs.name + getName());
        configureDevices(configs);

        this.robotToTurret = configs.robotToTurret;
        this.swervePoseSupplier = swervePoseSupplier;
    }

    private void configureDevices(TurretConfigs configs) {
        // Add motor/cancoder configs here
        motor = new MotorIOTalonFX(configs.motorConfigs);
        motor.withPIDGains(kPositionGains);
        motor.withCANCoder(configs.feedbackConfigs.FeedbackRemoteSensorID, configs.motorConfigs.kBus, configs.cancoderConfigs);
        motor.withFeedbackConfigs(configs.feedbackConfigs);
        motor.withStatorCurrentLimit(kCurrentLimit);
        motor.withMotionProfile(4, 4);
    }

    public Pose2d getTurretPose() {
        return swervePoseSupplier.get();
    }

    @Override
    public void periodicTelemetry() {

    }

    public Command track(Supplier<Translation2d> target) {
        return run(() -> {

        }).withName("Track");
    }

    public Command track(Translation2d target) {
        return this.track(() -> target);
    }


    public Command setDegrees(DoubleSupplier degrees) {
        return run(() -> {

        }).withName("SetDegrees");
    }

    public Command setDegrees(double degrees) {
        return this.setDegrees(() -> degrees);
    }

    public Command stop() {
        return run(() -> {

        }).withName("Stop");
    }
}