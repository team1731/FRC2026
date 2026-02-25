package frc.robot.subsystems.shooter.flywheel;

import frc.lib.frc1731.Utils;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.lib.frc1731.math.LoggedTunableNumber;
import frc.robot.subsystems.BaseSubsystem;

import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.*;

import edu.wpi.first.wpilibj2.command.Command;

public class FlywheelSubsystem extends BaseSubsystem {
    private MotorIOTalonFX leftMotor, rightMotor;
    private LoggedTunableNumber flywheelTunedSetpoint = new LoggedTunableNumber("Flywheel Setpoint RPS", 50, () -> true);

    private double rightTargetVelocity = 0.0;
    
    public FlywheelSubsystem(boolean enabled) {
        super(enabled);
        if (!isEnabled()) return;
        // leftMotor = new MotorIOTalonFX(kLeftFlywheelConfig);
        // leftMotor.withPIDGains(kVelocityGains);
        // leftMotor.withStatorCurrentLimit(kCurrentLimit);

        rightMotor = new MotorIOTalonFX(kRightFlywheelConfig);
        rightMotor.withPIDGains(kVelocityGains);
        rightMotor.withStatorCurrentLimit(kCurrentLimit);
        // super.withTolerance(kEpsilon);
        // super.withSysId(1d, 4d, 10d);
        // super.withSimulation(kSimConstants, kVelocityGains);
    }

    @Override
    public void periodicTelemetry() {
        // logger.log("Current Velocity RPS", getVelocity().in(RotationsPerSecond));
        // logger.log("Target Velocity RPS", getTargetVelocity().in(RotationsPerSecond));
        // logger.log("At Target Velocity", atTargetVelocity());

        // logger.log("Left Velocity", leftMotor.getVelocityRPS());
        logger.log("Right Velocity", rightMotor.getVelocityRPS());

    }

    public boolean atRightTargetVelocity() {
        return Utils.isWithin(rightMotor.getVelocityRPS(), rightTargetVelocity, 20);
    }

    // public Command tuneShotCommand() {
    //     return super.setVelocityCommand(() -> RotationsPerSecond.of(flywheelTunedSetpoint.get()));
    // }

    public Command setLeftFlywheelCommand(double rps) {
        return this.run(() -> {
            leftMotor.setVelocityRPS(rps);
        });
    }

    public Command setRightFlywheelCommand(double rps) {
        return this.run(() -> {
            rightTargetVelocity = rps;
            rightMotor.setVelocityRPS(rps);
        });
    }

    public Command setManualCommand(double percent) {
        return this.run(() -> {
            // leftMotor.setPercentOutput(percent);
            rightMotor.setPercentOutput(percent);
        });
    }

    public Command tuneableShotCommand() {
        return this.run(() -> {
            double rps = flywheelTunedSetpoint.get();
            leftMotor.setVelocityRPS(rps);
        });
    }

    public Command stopCommand() {
        return this.run(() -> {
            // leftMotor.setPercentOutput(0);
            rightMotor.setPercentOutput(0);
        });
    }
}