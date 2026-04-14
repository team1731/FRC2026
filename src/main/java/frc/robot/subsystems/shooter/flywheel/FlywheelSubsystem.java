package frc.robot.subsystems.shooter.flywheel;

import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.Utils;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.robot.subsystems.BaseSubsystem;

public class FlywheelSubsystem extends BaseSubsystem {
    private MotorIOTalonFX motor;
    private FlywheelIOInputs inputs = new FlywheelIOInputs();

    public FlywheelSubsystem(FlywheelConfiguration config, boolean enabled) {
        super(config.name(), config, enabled);
    }

    @Override
    protected void initializeHardware() {
        motor = new MotorIOTalonFX(((FlywheelConfiguration)config.get()).portConfig());
        motor.withPIDGains(kVelocityGains);
        motor.withStatorCurrentLimit(kCurrentLimit);
    }

    public boolean atTargetVelocity() {
        if (!isEnabled()) return true;
        return Utils.isWithin(motor.getVelocityRPS(), inputs.targetVelocity, kEpsilon);
    }

    @Override
    public void periodicTelemetry() {
        inputs.currentVelocity = motor.getVelocityRPS();
        inputs.atTargetVelocity = Utils.isWithin(inputs.currentVelocity, inputs.targetVelocity, kEpsilon);
     //   logger.processInputs(inputs);
        SmartDashboard.putNumber("Target Flywheel Velocity", inputs.targetVelocity);
    }

    public Command setVelocity(DoubleSupplier target) {
        return run(() -> {
            inputs.targetVelocity = Utils.clamp(target.getAsDouble(), kMaxVelocity);
            motor.setVelocityRPS(inputs.targetVelocity);
        }).withName("SetVelocity");
    }

    public Command setVelocity(double target) {
        return this.setVelocity(() -> target);
    }

    public Command setPercent(DoubleSupplier percent) {
        return run(() -> {
            inputs.targetVelocity = Utils.clamp(percent.getAsDouble(), 1.0) * kMaxVelocity;
            motor.setPercentOutput(percent.getAsDouble());
        }).withName("SetPercent");
    }

    public Command setPercent(double percent) {
        return this.setPercent(() -> percent);
    }

    public Command warmup() {
        return this.setVelocity(kWarmupVelocity);
    }

    public Command stop() {
        return this.setPercent(0)
        .withName("Stop");
    }
    
    public Command stopOnce() {
        return runOnce(() -> {
            motor.setPercentOutput(0d);
        });
    }
}
