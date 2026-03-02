package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.frc1731.Utils;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.lib.frc1731.math.LoggedTunableNumber;
import frc.robot.Ports;
import frc.robot.Robot;
import frc.robot.subsystems.BaseSubsystem;

import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.*;

import java.util.function.DoubleSupplier;
public class FlywheelSubsystem extends BaseSubsystem {
    private MotorIOTalonFX leftMotor, rightMotor;
    private LoggedTunableNumber leftSetpoint = new LoggedTunableNumber("Flywheel Setpoint Left", 50, () -> !DriverStation.isFMSAttached());
    private LoggedTunableNumber rightSetpoint = new LoggedTunableNumber("Flywheel Setpoint Right", 50, () -> !DriverStation.isFMSAttached());

    private double leftTargetRPS = 0d;
    private double rightTargetRPS = 0d;

    private double leftRPS = 0d;
    private double rightRPS = 0d;

    public FlywheelSubsystem(boolean enabled) {
        super(enabled);
        if (!isEnabled()) return;
        
        leftMotor = new MotorIOTalonFX(Ports.kLeftFlywheelConfig);
        leftMotor.withPIDGains(kVelocityGains);
        leftMotor.withStatorCurrentLimit(kCurrentLimit);

        rightMotor = new MotorIOTalonFX(Ports.kRightFlywheelConfig);
        rightMotor.withPIDGains(kVelocityGains);
        rightMotor.withStatorCurrentLimit(kCurrentLimit);

        Robot.IS_ENABLED.onTrue(new InstantCommand(() -> {
            leftMotor.setPercentOutput(0);
            rightMotor.setPercentOutput(0);
        }));
    }

    public boolean atLeftTargetVelocity() {
        return Utils.isWithin(leftRPS, leftTargetRPS, kEpsilon);
    }

    public boolean atRightTargetVelocity() {
        return Utils.isWithin(rightRPS, rightTargetRPS, kEpsilon);
    }

    public boolean atBothTargetVelocity() {
        return atLeftTargetVelocity() && atRightTargetVelocity();
    }

    @Override
    public void periodicTelemetry() {
        leftRPS = leftMotor.getVelocityRPS();
        rightRPS = rightMotor.getVelocityRPS();

        logger.log("Left/Velocity", leftRPS);
        logger.log("Left/Target Velocity", leftTargetRPS);
        logger.log("Left/At Target", atLeftTargetVelocity());

        logger.log("Right Velocity", rightRPS);
        logger.log("Right Target Velocity", rightTargetRPS);
        logger.log("At Right Target Velocity", atRightTargetVelocity());

        logger.log("Both At Target", atBothTargetVelocity());
    }

    public Command setLeft(DoubleSupplier rps) {
        return run(() -> {
            this.leftTargetRPS = rps.getAsDouble();
            leftMotor.setVelocityRPS(leftRPS);
        });
    }

    public Command setLeft(double rps) {
        return setLeft(() -> rps);
    }

    public Command setRight(DoubleSupplier rps) {
        return run(() -> {
            this.rightTargetRPS = rps.getAsDouble();
            rightMotor.setVelocityRPS(rightRPS);
        });
    }

    public Command setRight(double rps) {
        return setRight(() -> rps);
    }
    
    public Command setVelocity(DoubleSupplier left, DoubleSupplier right) {
        return run(() -> {
            this.leftTargetRPS = left.getAsDouble();
            this.rightTargetRPS = right.getAsDouble();

            leftMotor.setVelocityRPS(leftRPS);
            rightMotor.setVelocityRPS(rightRPS);
        });
    }

    public Command setVelocity(double left, double right) {
        return setVelocity(() -> left, () -> right);
    }

    public Command setTunedVelocity() {
        return setVelocity(leftSetpoint.get(), rightSetpoint.get());
    }

    public Command setPercent(double left, double right) {
        return run(() -> {
            leftMotor.setPercentOutput(left);
            leftMotor.setPercentOutput(right);
        });
    }

    public Command stop() {
        return this.run(() -> {
            leftMotor.setPercentOutput(0);
            rightMotor.setPercentOutput(0);
        }).withTimeout(0);
    }
}
