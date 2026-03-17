package frc.robot.subsystems.indexer;

import static frc.robot.subsystems.indexer.IndexerConstants.*;

import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.lib.frc1731.subsystem.BaseVelocitySubsystem;
import frc.robot.Ports;

import static edu.wpi.first.units.Units.*;

public class IndexerSubsystem extends BaseVelocitySubsystem<MotorIOTalonFX>{
    public IndexerSubsystem(boolean enabled) {
        super(enabled);
    }

    @Override
    protected void initializeHardware() {
        motor = new MotorIOTalonFX(Ports.kIndexerConfig);
        motor.withPIDGains(kPIDGains);
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Velocity", getVelocity().in(RotationsPerSecond));
        logger.log("Target Velocity", getTargetVelocity().in(RotationsPerSecond));
    }
}
