package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFXS;
import frc.robot.subsystems.BaseSubsystem;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.hood.HoodConstants.*;

public class HoodSubsystem extends BaseSubsystem {
    private MotorIOTalonFXS leftMotor, rightMotor;
    private Angle targetAngle = Degrees.zero();

    public HoodSubsystem(boolean enabled) {
        super(enabled);
        if (!isEnabled()) return;
        leftMotor = new MotorIOTalonFXS(kLeftHoodConfig);
        rightMotor = new MotorIOTalonFXS(kRightHoodConfig);
        
        leftMotor.withPIDGains(kPositionGains);
        rightMotor.withPIDGains(kPositionGains);

        leftMotor.resetEncoderPosition(kStartRotations.in(Rotations));
        rightMotor.resetEncoderPosition(kStartRotations.in(Rotations));

        leftMotor.setSoftLimits(kConverter.toMotor(kStartAngle).in(Rotations), kConverter.toMotor(kMaxAngle).in(Rotations));
        rightMotor.setSoftLimits(kConverter.toMotor(kStartAngle).in(Rotations), kConverter.toMotor(kMaxAngle).in(Rotations));
    }

    public Angle getMotorAngle() {
        return Rotations.of(leftMotor.getRotations());
    }

    public Angle getHoodAngle() {
        return kConverter.toMechanism(Rotations.of(leftMotor.getRotations()));
    }

    public Angle getTargetAngle() {
        return targetAngle;
    }

    // @SuppressWarnings("unused")
    // private void setVoltage(Voltage volts) {
    //     leftMotor.setVoltage(volts.in(Volts));
    //     if (Robot.isSimulation()) sim.setVoltage(volts);
    // }

    public boolean atTargetAngle() {
        return getHoodAngle().isNear(targetAngle, kEpsilon);
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Position", getMotorAngle().in(Rotations));
        logger.log("Current Degrees", getHoodAngle().in(Degrees));
        logger.log("Target Position", kConverter.toMotor(getTargetAngle()).in(Rotations));
        logger.log("Target Degrees", getTargetAngle().in(Degrees));
        logger.log("At Target Angle", atTargetAngle());
    }

    public Command setLeftHoodCommand(double targetDegrees) {
        return this.run(() -> {
            leftMotor.setPosition(kConverter.toMotor(Degrees.of(targetDegrees)).in(Rotations));
        });
    }

    public Command setRightHoodCommand(double targetDegrees) {
        return this.run(() -> {
            rightMotor.setPosition(kConverter.toMotor(Degrees.of(targetDegrees)).in(Rotations));
        });
    }

    public Command stowHoodCommand() {
        return setLeftHoodCommand(kStartAngle.in(Degrees)).alongWith(setRightHoodCommand(kCurrentLimit));
    }

    public Command setManualCommand(double left, double right) {
        return run(() -> {
            leftMotor.setPercentOutput(left);
            rightMotor.setPercentOutput(right);
        });
    }
}