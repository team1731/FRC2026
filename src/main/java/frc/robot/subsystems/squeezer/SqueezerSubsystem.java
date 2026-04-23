package frc.robot.subsystems.squeezer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.hardware.motor.rev.MotorIOSparkMax;
import frc.robot.Ports;
import frc.robot.subsystems.BaseSubsystem;

public class SqueezerSubsystem extends BaseSubsystem {
    private MotorIOSparkMax motor;
    private SqueezerIOInputsAutoLogged inputs = new SqueezerIOInputsAutoLogged();

    public SqueezerSubsystem(boolean enabled) {
        super(enabled);
    }

    @Override
    protected void initializeHardware() {
        motor = new MotorIOSparkMax(Ports.kSqueezerConfig);
        motor.withStatorCurrentLimit(SqueezerConstants.kCurrentLimit);
        motor.withPIDGains(SqueezerConstants.kPositionGains);
        motor.setSoftLimits(0, SqueezerConstants.kMaxPosition);
    }

    @Override
    public void periodicTelemetry() {
        inputs.currentPosition = motor.getRotations();
        inputs.atTargetPosition = Math.abs(inputs.currentPosition - inputs.targetPosition) < 0.01;
        logger.processInputs(inputs);
    }

    public Command raise() {
        return run(() -> {
            // inputs.targetPosition = SqueezerConstants.kMaxPosition;
            // motor.setPosition(inputs.targetPosition);
            motor.setPercentOutput(0.1);
        });
    }
    
    public Command squeeze() {
        return run(() -> {
            motor.setPercentOutput(-0.1);
        });
    }
}