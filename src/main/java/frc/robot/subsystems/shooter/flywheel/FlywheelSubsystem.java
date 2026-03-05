package frc.robot.subsystems.shooter.flywheel;

import frc.lib.frc1731.Utils;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.lib.frc1731.math.LoggedTunableNumber;
import frc.robot.Robot;
import frc.robot.subsystems.BaseSubsystem;

import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class FlywheelSubsystem extends BaseSubsystem {
    private MotorIOTalonFX leftMotor, rightMotor;
    // private LoggedTunableNumber flywheelTunedSetpoint = new LoggedTunableNumber("Flywheel Setpoint RPS", 50, () -> true);

    private double leftTargetVelocity = 0.0;
    private double rightTargetVelocity = 0.0;
    
    public FlywheelSubsystem(boolean enabled) {
        super(enabled);
        if (!isEnabled()) return;
        
        leftMotor = new MotorIOTalonFX(kLeftFlywheelConfig);
        leftMotor.withPIDGains(kVelocityGains);
        leftMotor.withStatorCurrentLimit(kCurrentLimit);

        rightMotor = new MotorIOTalonFX(kRightFlywheelConfig);
        rightMotor.withPIDGains(kVelocityGains);
        rightMotor.withStatorCurrentLimit(kCurrentLimit);
        // super.withTolerance(kEpsilon);
        // super.withSysId(1d, 4d, 10d);
        // super.withSimulation(kSimConstants, kVelocityGains);

        Robot.IS_ENABLED.onTrue(new InstantCommand(() -> {
            leftMotor.setPercentOutput(0);
            rightMotor.setPercentOutput(0);
        }));
    }

    @Override
    public void periodicTelemetry() {
        // logger.log("Current Velocity RPS", getVelocity().in(RotationsPerSecond));
        // logger.log("Target Velocity RPS", getTargetVelocity().in(RotationsPerSecond));
        // logger.log("At Target Velocity", atTargetVelocity());

        logger.log("Left Velocity", leftMotor.getVelocityRPS());
        logger.log("Right Velocity", rightMotor.getVelocityRPS());
        logger.log("Left Target Velocity", leftTargetVelocity);
        logger.log("Right Target Velocity", rightTargetVelocity);
        logger.log("At Left Target Velocity", atLeftTargetVelocity());
        logger.log("At Right Target Velocity", atRightTargetVelocity());
        
    }

    public boolean atLeftTargetVelocity() {
        return Utils.isWithin(leftMotor.getVelocityRPS(), leftTargetVelocity, 2);
    }

    public boolean atRightTargetVelocity() {
        return Utils.isWithin(rightMotor.getVelocityRPS(), rightTargetVelocity, 2);
    }

    public boolean atBothTargetVelocity() {
        return atLeftTargetVelocity() && atRightTargetVelocity();
    }

    // public Command tuneShotCommand() {
    //     return setFlywheelVelocityCommand(flywheelTunedSetpoint.get(), flywheelTunedSetpoint.get());
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

    public Command setFlywheelPercentCommand(double left, double right) {
        return this.run(() -> {
            this.leftTargetVelocity = left * 100;
            this.rightTargetVelocity = right * 100;
            leftMotor.setPercentOutput(left);
            rightMotor.setPercentOutput(right);
        });
    }

    public Command setFlywheelVelocityCommand(double left, double right) {
        return this.run(() -> {
            this.leftTargetVelocity = left;
            this.rightTargetVelocity = right;
            leftMotor.setVelocityRPS(left);
            rightMotor.setVelocityRPS(right);
        });
    }

    public Command setFlywheelCommand(DoubleSupplier velocity) {
        return this.run(() -> {
            this.leftTargetVelocity = velocity.getAsDouble();
            this.rightTargetVelocity = velocity.getAsDouble();
            leftMotor.setVelocityRPS(velocity.getAsDouble());
            rightMotor.setVelocityRPS(velocity.getAsDouble());
        });
    }

    public Command tuneableShotCommand() {
        return this.run(() -> {
            // double rps = flywheelTunedSetpoint.get();
            // leftMotor.setVelocityRPS(rps);
            // rightMotor.setVelocityRPS(rps);
        });
    }

    public Command stopCommand() {
        return this.run(() -> {
            leftMotor.setPercentOutput(0);
            rightMotor.setPercentOutput(0);
        });
    }
}