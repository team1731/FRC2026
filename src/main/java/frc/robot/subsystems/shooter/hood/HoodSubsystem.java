package frc.robot.subsystems.shooter.hood;

import static frc.robot.subsystems.shooter.hood.HoodConstants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.Utils;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFXS;
import frc.robot.subsystems.BaseSubsystem;

public class HoodSubsystem extends BaseSubsystem {
    private MotorIOTalonFXS motor;
    private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

    public HoodSubsystem(HoodConfiguration config, boolean enabled) {
        super(config.name(), config, enabled);
    }

    @Override
    protected void initializeHardware() {
        motor = new MotorIOTalonFXS(((HoodConfiguration)config.get()).portConfig());
        motor.withPIDGains(kPositionGains);
        motor.withStatorCurrentLimit(kCurrentLimit);
        motor.withMotionProfile(kMaxVelocity, kMaxAcceleration);
        motor.setSoftLimits(kMinRotations, kMaxRotations);
        motor.resetEncoderPosition(0);
    }

    public boolean atTarget() {
        if (!isEnabled()) return true;
        return inputs.atTarget;
    }

    @Override
    public void periodicTelemetry() {
        inputs.motorRotations = motor.getRotations();
        inputs.atTarget = Utils.isWithin(inputs.motorRotations, inputs.targetRotations, kEpsilon);
        logger.processInputs(inputs);
    }

    public Command setRotations(DoubleSupplier target) {
        return run(() -> {
            inputs.targetRotations = Utils.clamp(target.getAsDouble(), kMinRotations, kMaxRotations);
            motor.setPosition(inputs.targetRotations);
        }).withName("SetRotations");
    }

    public Command setRotations(double target) {
        return setRotations(() -> target);
    }

    public Command stow() {
        return setRotations(kMinRotations)
        .withName("Stow");
    }

    public Command stowOnce() {
        return runOnce(() -> {
            inputs.targetRotations = kMinRotations;
            this.motor.setPosition(inputs.targetRotations);
        }).withName("Stow");
    }
}