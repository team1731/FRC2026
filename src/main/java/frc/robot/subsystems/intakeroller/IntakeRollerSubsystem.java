package frc.robot.subsystems.intakeroller;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.hardware.motor.io.MotorIOTalonFX;
import frc.lib.frc1731.subsystem.BaseVelocitySubsystem;

public class IntakeRollerSubsystem extends BaseVelocitySubsystem<MotorIOTalonFX> {
    private final IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();
    public IntakeRollerSubsystem() {
        super(IntakeRollerConstants.getIO());
    }

    @Override
    public void periodicTelemetry() {
        inputs.currentVelocity = getVelocity().in(RotationsPerSecond);
        inputs.setpointVelocity = getSetpoint().in(RotationsPerSecond);
        logger.processInputs(inputs);
    }

    public Command intake() {
        return setVelocity(IntakeRollerConstants.kIntakeVelocity);
    }
}