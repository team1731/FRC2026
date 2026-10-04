package frc.robot.subsystems.intakeroller;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.hardware.motor.io.MotorIOTalonFX;
import frc.lib.frc1731.hardware.motor.io.request.VelocityRequest;
import frc.lib.frc1731.subsystem.BaseVelocityInputsAutoLogged;
import frc.lib.frc1731.subsystem.BaseVelocitySubsystem;

public class IntakeRollerSubsystem extends BaseVelocitySubsystem<MotorIOTalonFX, BaseVelocityInputsAutoLogged> {
    public static final VelocityRequest kIntakeRequest = new VelocityRequest(RotationsPerSecond.of(0));
    public IntakeRollerSubsystem() {
        super(IntakeRollerConstants.getIO(), new BaseVelocityInputsAutoLogged());
    }

    @Override
    protected BaseVelocityInputsAutoLogged updateInputs(BaseVelocityInputsAutoLogged inputs) {
        inputs.currentVelocity = getVelocity();
        inputs.setpointVelocity = getSetpoint();
        inputs.atSetpoint = atSetpoint();
        return inputs;
    }

    public Command intake() {
        return applyRequest(kIntakeRequest);
    }
}