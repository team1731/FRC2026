package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.wpilibj2.command.*;
import frc.lib.frc1731.Utils;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFXS;
import frc.robot.subsystems.BaseSubsystem;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.hood.HoodConstants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;

public class HoodSubsystem extends BaseSubsystem {
    private MotorIOTalonFXS leftMotor, rightMotor;
    private double leftTargetDegrees = 0;
    private double rightTargetDegrees = 0;

    public HoodSubsystem(boolean enabled) {
        super(enabled);
        if (!isEnabled()) return;
        leftMotor = new MotorIOTalonFXS(kLeftHoodConfig);
        rightMotor = new MotorIOTalonFXS(kRightHoodConfig);

        leftMotor.withPIDGains(kPositionGains);
        rightMotor.withPIDGains(kPositionGains);

        leftMotor.withMotionMagicConfigs(new MotionMagicConfigs()
            .withMotionMagicAcceleration(50)
            .withMotionMagicCruiseVelocity(120)
        );

        rightMotor.withMotionMagicConfigs(new MotionMagicConfigs()
            .withMotionMagicAcceleration(50)
            .withMotionMagicCruiseVelocity(120)
        );

        leftMotor.setDynamicMotionMagicSpeeds(50, 120);
        rightMotor.setDynamicMotionMagicSpeeds(50, 120);

        leftMotor.resetEncoderPosition(0);
        rightMotor.resetEncoderPosition(0);

        // leftMotor.setSoftLimits(0, 7);
        // rightMotor.setSoftLimits(0, 7);

        // Robot.IS_ENABLED.onTrue(new InstantCommand(() -> {
        //     leftMotor.setPercentOutput(0);
        //     rightMotor.setPercentOutput(0);
        // }));
    }

    public double getLeftHoodDegrees() {
        return kConverter.toMechanism(Rotations.of(leftMotor.getRotations())).in(Degrees);
    }

    public double getRightHoodDegrees() {
        return kConverter.toMechanism(Rotations.of(rightMotor.getRotations())).in(Degrees);
    }

    public double getLeftTargetDegrees() {
        // return kConverter.toMechanism(Rotations.of(leftTargetDegrees)).in(Degrees);
        return leftTargetDegrees;
    }

    public double getRightTargetDegrees() {
        // return kConverter.toMechanism(Rotations.of(rightTargetDegrees)).in(Degrees);
        return rightTargetDegrees;
    }

    public boolean atLeftTarget() {
        return Utils.isWithin(getLeftHoodDegrees(), getLeftTargetDegrees(), kEpsilon.in(Degrees));
    }

    public boolean atRightTarget() {
        return Utils.isWithin(getRightHoodDegrees(), getRightTargetDegrees(), kEpsilon.in(Degrees));
    }

    public boolean atBothTarget() {
        return atLeftTarget() && atRightTarget();
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Left Hood Position", getLeftHoodDegrees());
        logger.log("Right Hood Position", getRightHoodDegrees());
        logger.log("Left Hood Target Position", getLeftTargetDegrees());
        logger.log("Right Hood Target Position", getRightTargetDegrees());
        logger.log("Left Hood At Target Position", atLeftTarget());
        logger.log("Right Hood At Target Position", atRightTarget());
    }

    public Command setLeftHoodCommand(double targetDegrees) {
        return this.run(() -> {
            this.leftTargetDegrees = kConverter.toMotor(Degrees.of(targetDegrees)).in(Rotations);
            leftMotor.setPosition(targetDegrees);
        });
    }

    public Command setRightHoodCommand(double targetDegrees) {
        return this.run(() -> {
            this.rightTargetDegrees = kConverter.toMotor(Degrees.of(targetDegrees)).in(Rotations);
            rightMotor.setPosition(targetDegrees);
        });
    }

    public Command setHoodCommand(double left, double right) {
        return this.run(() -> {
            this.leftTargetDegrees = left;
            this.rightTargetDegrees = right;
            
            leftMotor.setPosition(left);
            rightMotor.setPosition(right);
        });
    }

    public Command setHoodCommand(DoubleSupplier left, DoubleSupplier right) {
        return this.run(() -> {
            this.leftTargetDegrees = left.getAsDouble();
            this.rightTargetDegrees = right.getAsDouble();
            
            leftMotor.setPosition(left.getAsDouble());
            rightMotor.setPosition(right.getAsDouble());
        });
    }

    public Command setHoodCommand(DoubleSupplier rotations) {
        return this.run(() -> {
            this.leftTargetDegrees = rotations.getAsDouble();
            this.rightTargetDegrees = rotations.getAsDouble();
            
            leftMotor.setPosition(rotations.getAsDouble());
            rightMotor.setPosition(rotations.getAsDouble());
        });
    }

    public Command stowHoodCommand() {
        return setHoodCommand(kStartRotations.in(Rotations), kStartRotations.in(Rotations));
    }

    public Command driveManualCommand(double left, double right) {
        return run(() -> {
            leftMotor.setPercentOutput(left);
            rightMotor.setPercentOutput(right);
        });
    }
}