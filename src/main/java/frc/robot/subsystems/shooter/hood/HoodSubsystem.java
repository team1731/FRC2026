package frc.robot.subsystems.shooter.hood;

import frc.lib.frc1731.Utils;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFXS;
import frc.lib.frc1731.math.LoggedTunableNumber;
import frc.robot.Ports;
import frc.robot.subsystems.BaseSubsystem;

import static frc.robot.subsystems.shooter.hood.HoodConstants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class HoodSubsystem extends BaseSubsystem {
    private MotorIOTalonFXS leftMotor, rightMotor;
    private double leftTargetRotations = 0;
    private double rightTargetRotations = 0;

    private double leftRotations = 0;
    private double rightRotations = 0;

    private LoggedTunableNumber leftSetpoint = new LoggedTunableNumber("Hood Left Setpoint Rotations", 3, () -> !DriverStation.isFMSAttached());
    private LoggedTunableNumber rightSetpoint = new LoggedTunableNumber("Hood Right Setpoint Rotations", 3, () -> !DriverStation.isFMSAttached());

    public HoodSubsystem(boolean enabled) {
        super(enabled);
        if (!isEnabled()) return;
        leftMotor = new MotorIOTalonFXS(Ports.kLeftHoodConfig);
        leftMotor.withPIDGains(kPositionGains);
        leftMotor.withMotionMagicConfigs(kMotionMagicConfigs);
        leftMotor.resetEncoderPosition(0);
        leftMotor.setSoftLimits(-0.1, 7);
        // leftMotor.setDynamicMotionMagicSpeeds(50, 50);

        rightMotor = new MotorIOTalonFXS(Ports.kRightHoodConfig);
        rightMotor.withPIDGains(kPositionGains);
        rightMotor.withMotionMagicConfigs(kMotionMagicConfigs);
        rightMotor.resetEncoderPosition(0);
        rightMotor.setSoftLimits(-0.1, 7);
        // rightMotor.setDynamicMotionMagicSpeeds(50, 50);
    }

    public boolean atLeftTargetRotations() {
        return Utils.isWithin(leftRotations, leftTargetRotations, kEpsilon);
    }

    public boolean atRightTargetRotations() {
        return Utils.isWithin(rightRotations, rightTargetRotations, kEpsilon);
    }

    public boolean atBothTargetRotations() {
        return atLeftTargetRotations() && atRightTargetRotations();
    }

    @Override
    public void periodicTelemetry() {
        leftRotations = leftMotor.getRotations();
        rightRotations = rightMotor.getRotations();

        logger.log("Left/Rotations", leftRotations);
        logger.log("Left/Target Rotations", leftTargetRotations);
        logger.log("Left/At Target", atLeftTargetRotations());

        logger.log("Right/Rotations", rightRotations);
        logger.log("Right/Target Rotations", rightTargetRotations);
        logger.log("Right/At Target", atRightTargetRotations());

        logger.log("At Both Target", atBothTargetRotations());
    }

    public Command setLeft(DoubleSupplier rotations) {
        return run(() -> {
            leftTargetRotations = rotations.getAsDouble();
            leftMotor.setPosition(leftTargetRotations);
        });
    }

    public Command setLeft(double rotations) {
        return setLeft(() -> rotations);
    }

    public Command setRight(DoubleSupplier rotations) {
        return run(() -> {
            rightTargetRotations = rotations.getAsDouble();
            rightMotor.setPosition(rightTargetRotations);
        });
    }

    public Command setRight(double rotations) {
        return setRight(() -> rotations);
    }

    public Command setRotations(DoubleSupplier left, DoubleSupplier right) {
        return run(() -> {
            leftTargetRotations = left.getAsDouble();
            rightTargetRotations = right.getAsDouble();

            leftMotor.setPosition(leftTargetRotations);
            rightMotor.setPosition(rightTargetRotations);
        });
    }

    public Command setRotations(double left, double right) {
        return setRotations(() -> left, () -> right);
    }

    public Command stow() {
        return setRotations(0, 0);
    }

    public Command setTunedRotations() {
        return setRotations(leftSetpoint.get(), rightSetpoint.get());
    }

    public Command driveManual(double left, double right) {
        return run(() -> {
            leftMotor.setPercentOutput(left);
            rightMotor.setPercentOutput(right);
        });
    }
    
    public Command stop() {
        return driveManual(0, 0).withTimeout(0);
    }
}