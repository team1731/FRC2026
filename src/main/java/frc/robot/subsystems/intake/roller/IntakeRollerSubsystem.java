package frc.robot.subsystems.intake.roller;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.lib.frc1731.subsystem.VelocitySubsystem;
import frc.robot.Ports;

public class IntakeRollerSubsystem extends VelocitySubsystem<MotorIOTalonFX>{
    private IntakeRollerIOInputsAutoLogged inputs = new IntakeRollerIOInputsAutoLogged();

    public IntakeRollerSubsystem(boolean enabled){
        super(enabled);
    }

    @Override
    protected void initializeHardware() {
        motor = new MotorIOTalonFX(Ports.kIntakeRollerConfig);
        motor.withPIDGains(kRollerGains);
        motor.withStatorCurrentLimit(kRollerCurrentLimit);
    }

    @Override
    public void periodicTelemetry() {
        inputs.currentVelocity = getVelocity().in(RotationsPerSecond);
        inputs.targetVelocity = getTargetVelocity().in(RotationsPerSecond);
        logger.processInputs(inputs);
    }
}