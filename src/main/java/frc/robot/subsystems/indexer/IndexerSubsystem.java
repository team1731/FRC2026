package frc.robot.subsystems.indexer;

import frc.lib.frc1731.Utils;
import frc.lib.frc1731.hardware.motor.MotorConstants;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.robot.Ports;
import frc.robot.subsystems.BaseSubsystem;

import static frc.robot.subsystems.indexer.IndexerConstants.*;

import edu.wpi.first.wpilibj2.command.Command;

public class IndexerSubsystem extends BaseSubsystem {
    private MotorIOTalonFX leftMotor, rightMotor;
    private IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public IndexerSubsystem(boolean enabled) {
        super(enabled);
    }

    @Override
    protected void initializeHardware() {
        leftMotor = new MotorIOTalonFX(Ports.kIndexerLeftConfig);
        rightMotor = new MotorIOTalonFX(Ports.kIndexerRightConfig);

        leftMotor.withPIDGains(kPIDGains);
        rightMotor.withPIDGains(kPIDGains);

        leftMotor.withStatorCurrentLimit(kCurrentLimit);
        rightMotor.withStatorCurrentLimit(kCurrentLimit);
    }

    @Override
    public void periodicTelemetry() {
        inputs.currentVelocityLeft = leftMotor.getVelocityRPS();
        inputs.currentVelocityRight = rightMotor.getVelocityRPS();

        inputs.atTargetVelocityLeft = Utils.isWithin(inputs.currentVelocityLeft, inputs.targetVelocityLeft, 1);
        inputs.atTargetVelocityRight = Utils.isWithin(inputs.currentVelocityRight, inputs.targetVelocityRight, 1);

        logger.processInputs(inputs);
    }

    public Command setPercent(double left, double right) {
        return run(() -> {
            inputs.targetVelocityLeft = right * MotorConstants.KRAKEN_X60.MAX_VELOCITY_RPM / 60.0;
            inputs.targetVelocityRight = left * MotorConstants.KRAKEN_X60.MAX_VELOCITY_RPM / 60.0;

            this.rightMotor.setPercentOutput(left);
            this.leftMotor.setPercentOutput(right);
        });
    }

    public Command setVelocity(double right, double left) {
        return run(() -> {
            inputs.targetVelocityLeft = left;
            inputs.targetVelocityRight = right;

            this.leftMotor.setVelocityRPS(left);
            this.rightMotor.setVelocityRPS(right);
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