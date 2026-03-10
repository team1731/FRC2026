package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.hood.HoodConstants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.Utils;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFXS;
import frc.robot.subsystems.BaseSubsystem;

public class HoodSubsystem extends BaseSubsystem {
    private MotorIOTalonFXS motor;
    private double targetRotations = 0;

    public HoodSubsystem(HoodConfiguration config, boolean enabled) {
        super(config.name(), enabled);
        if (!isEnabled()) return;
        initializeHardware(config);
    }

    private void initializeHardware(HoodConfiguration config) {
        motor = new MotorIOTalonFXS(config.portConfig());
        motor.withPIDGains(kPositionGains);
        motor.withStatorCurrentLimit(kCurrentLimit);
        motor.withMotionProfile(kMaxVelocity, kMaxAcceleration);
        motor.setSoftLimits(kMinRotations, kMaxRotations);
        motor.resetEncoderPosition(0);
        // TODO - SET FEEDBACK CONFIGS AND MAYBE CANCODER DEPENDING ON IF WE EVEN WANT IT
    }

    public boolean atTarget() {
        if (!isEnabled()) return true;
        return Utils.isWithin(motor.getRotations(), targetRotations, kEpsilon);
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Rotations", motor.getRotations());
        logger.log("Target Rotations", targetRotations);
        logger.log("At Target", atTarget());
    }

    public Command setRotations(DoubleSupplier target) {
        return run(() -> {
            this.targetRotations = Utils.clamp(target.getAsDouble(), kMinRotations, kMaxRotations);
            motor.setPosition(targetRotations);
        }).withName("SetRotations");
    }

    public Command setRotations(double target) {
        return setRotations(() -> target);
    }

    public Command stow() {
        return setRotations(0)
        .withName("Stow");
    }
}