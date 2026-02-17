package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.robot.subsystems.BaseSubsystem;

public class IntakePivotSubsystem extends BaseSubsystem {
    private MotorIOTalonFX motor;

    public IntakePivotSubsystem(boolean enabled) {
        super(enabled);
        if (!isEnabled()) return;
        motor = new MotorIOTalonFX(IntakeConstants.kPivotMotorConfig);
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Rotations", motor.getRotations());
    }
    
    public Command setManualCommand(double percentOutput) {
        return run(() -> motor.setPercentOutput(percentOutput));
    }

}