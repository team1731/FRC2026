package frc.robot.subsystems.feeder;

import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.lib.frc1731.sim.SimpleVelocitySim.SimConstants;
import frc.lib.frc1731.subsystem.VelocitySubsystem;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.feeder.FeederConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;

public class IndexerSubsystem extends VelocitySubsystem<MotorIOTalonFX>{

    public IndexerSubsystem(boolean enabled) {
        super(enabled);
    }

    @Override
    protected void initializeHardware() {
        motor = new MotorIOTalonFX(kMotorConfig);
        motor.withPIDGains(kPIDGains);
        // motor.withFollower(new MotorIOTalonFX(kFollowerConfig));
        this.withSimulation(new SimConstants(DCMotor.getKrakenX60(1), 1.0, Inches.of(1), Pounds.of(0.1)), kPIDGains);

    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Velocity", getVelocity().in(RotationsPerSecond));
        logger.log("Target Velocity", getTargetVelocity().in(RotationsPerSecond));
    }
    
}
