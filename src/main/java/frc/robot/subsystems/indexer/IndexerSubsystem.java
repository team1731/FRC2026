package frc.robot.subsystems.indexer;

import frc.lib.frc1731.Utils;
import frc.lib.frc1731.hardware.motor.MotorConstants;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.robot.Ports;
import frc.robot.subsystems.BaseSubsystem;

import static frc.robot.subsystems.indexer.IndexerConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class IndexerSubsystem extends BaseSubsystem {
    private MotorIOTalonFX motor;
    private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public IndexerSubsystem(boolean enabled) {
        super(enabled);
    }

    @Override
    protected void initializeHardware() {
        motor = new MotorIOTalonFX(Ports.kIndexerFloorConfig);
        motor.withPIDGains(kPIDGains);
        // motor.withStatorCurrentLimit(kCurrentLimit);
    }

    @Override
    public void periodicTelemetry() {
        inputs.currentVelocity = motor.getVelocityRPS();
        inputs.atTargetVelocity = Utils.isWithin(inputs.currentVelocity, inputs.targetVelocity, 1);
     //   logger.processInputs(inputs);
        SmartDashboard.putNumber("INDEXER RPS", inputs.currentVelocity);
    }

    public Command setPercent(double setpoint) {
        return run(() -> {
            inputs.targetVelocity = setpoint * MotorConstants.KRAKEN_X60.MAX_VELOCITY_RPM / 60.0;
            this.motor.setPercentOutput(setpoint);
        });
    }

    public Command setVelocity(double setpoint) {
        return run(() -> {
            inputs.targetVelocity = setpoint;
            this.motor.setVelocityRPS(inputs.targetVelocity);
        });
    }

    public Command feed() {
        return setVelocity(kFeedRPS);
        // return setPercent(0.6);
    }

    public Command eject() {
        return setVelocity(kEjectRPS);
    }

    public Command stop() {
        return setPercent(0);
    }
}
