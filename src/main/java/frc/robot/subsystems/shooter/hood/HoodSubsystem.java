package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.wpilibj2.command.*;
import frc.lib.frc1731.Utils;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFXS;
import frc.robot.subsystems.BaseSubsystem;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.hood.HoodConstants.*;

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
        
        leftMotor.setSoftLimits(0, 7);
        rightMotor.setSoftLimits(0, 7);

        leftMotor.withMotionMagicConfigs(new MotionMagicConfigs()
            .withMotionMagicAcceleration(50)
            .withMotionMagicCruiseVelocity(50)
            .withMotionMagicJerk(100)
        );

        rightMotor.withMotionMagicConfigs(new MotionMagicConfigs()
            .withMotionMagicAcceleration(50)
            .withMotionMagicCruiseVelocity(50)
            .withMotionMagicJerk(100)
        );

        leftMotor.setDynamicMotionMagicSpeeds(50, 50);
        rightMotor.setDynamicMotionMagicSpeeds(50, 50);

        leftMotor.resetEncoderPosition(kStartRotations.in(Rotations));
        rightMotor.resetEncoderPosition(kStartRotations.in(Rotations));
    }

    public double getLeftHoodDegrees() {
        return kConverter.toMechanism(Rotations.of(leftMotor.getRotations())).in(Degrees);
    }

    public double getRightHoodDegrees() {
        return kConverter.toMechanism(Rotations.of(rightMotor.getRotations())).in(Degrees);
    }

    public double getLeftTargetDegrees() {
        return kConverter.toMechanism(Rotations.of(leftTargetDegrees)).in(Degrees);
    }

    public double getRightTargetDegrees() {
        return kConverter.toMechanism(Rotations.of(rightTargetDegrees)).in(Degrees);
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
            this.leftTargetDegrees = kConverter.toMotor(Degrees.of(left)).in(Rotations);
            this.rightTargetDegrees = kConverter.toMotor(Degrees.of(right)).in(Rotations);
            
            leftMotor.setPosition(left);
            rightMotor.setPosition(right);
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