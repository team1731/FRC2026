package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.lib.frc1731.subsystem.VelocitySubsystem;
import frc.robot.Ports;

public class IntakeRollerSubsystem extends VelocitySubsystem<MotorIOTalonFX>{
    public IntakeRollerSubsystem(boolean enabled){
        super(enabled);
        // this.withSimulation(new SimConstants(DCMotor.getKrakenX60(1), 1.5, Inches.of(1), Pounds.of(0.1)), kPIDGains);
    }

    @Override
    protected void initializeHardware() {
        motor = new MotorIOTalonFX(Ports.kIntakeRollerConfig);
        motor.withPIDGains(kRollerGains);
        motor.withStatorCurrentLimit(40);
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Velocity", getVelocity().in(RotationsPerSecond));
        logger.log("Target Velocity", getTargetVelocity().in(RotationsPerSecond));
    }

}
