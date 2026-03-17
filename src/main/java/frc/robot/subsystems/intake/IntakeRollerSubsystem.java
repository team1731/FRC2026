package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.lib.frc1731.subsystem.BaseVelocitySubsystem;
import frc.robot.Ports;

public class IntakeRollerSubsystem extends BaseVelocitySubsystem<MotorIOTalonFX>{
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
        logger.log("Current Velocity", getVelocity().in(RotationsPerSecond));
        logger.log("Target Velocity", getTargetVelocity().in(RotationsPerSecond));
    }

}
