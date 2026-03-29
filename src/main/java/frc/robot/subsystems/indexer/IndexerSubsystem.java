package frc.robot.subsystems.indexer;

import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.robot.Ports;
import frc.robot.subsystems.BaseSubsystem;

import static frc.robot.subsystems.indexer.IndexerConstants.*;

import edu.wpi.first.wpilibj2.command.Command;

public class IndexerSubsystem extends BaseSubsystem {
    private MotorIOTalonFX bottomMotor, topMotor;

    public IndexerSubsystem(boolean enabled) {
        super(enabled);
    }

    @Override
    protected void initializeHardware() {
        bottomMotor = new MotorIOTalonFX(Ports.kIndexerBottomConfig);
        topMotor = new MotorIOTalonFX(Ports.kIndexerTopConfig);

        bottomMotor.withPIDGains(kPIDGains);
        topMotor.withPIDGains(kPIDGains);
    }

    @Override
    public void periodicTelemetry() {}

    public Command setPercent(double percentTop, double percentBottom) {
        return run(() -> {
            this.topMotor.setPercentOutput(percentTop);
            this.bottomMotor.setPercentOutput(percentBottom);
        });
    }

    public Command setVelocity(double topRPS, double bottomRPS) {
        return run(() -> {
            this.topMotor.setVelocityRPS(topRPS);
            this.bottomMotor.setVelocityRPS(bottomRPS);
        });
    }

    public Command index() {
        return setPercent(1.0, 1.0);
    }

    public Command unjam() {
        return setPercent(-1.0, -1.0);
    }

    public Command stop() {
        return setPercent(0, 0);
    }
}