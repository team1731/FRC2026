package frc.lib.frc1731.hardware.motor.rev;


import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.Utils;
import frc.lib.frc1731.hardware.motor.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * Wrapper class for rev motors that use a spark motor controller
 */
public class MotorIOSparkFlex extends MotorIO {
    protected SparkFlex motor;
    protected SparkClosedLoopController motorCtrl;
    protected RelativeEncoder encoder;
    protected SparkFlexConfig config;

    public MotorIOSparkFlex(PortConfig config) {
        super(config);
        this.motor = new SparkFlex(config.kPort, MotorType.kBrushless);
        this.motorCtrl = motor.getClosedLoopController();
        this.encoder = motor.getEncoder();
        
        this.config = new SparkFlexConfig();
        this.config.inverted(config.kInverted);
    }

    @Override
    public void withMotionProfile(double maxVelocity, double maxAcceleration) {
        this.config.closedLoop.maxMotion
            .cruiseVelocity(maxVelocity, ClosedLoopSlot.kSlot0)
            .maxAcceleration(maxAcceleration, ClosedLoopSlot.kSlot0)
            .positionMode(com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot0)
            .allowedProfileError(0, ClosedLoopSlot.kSlot0)
        ;

        this.motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void withPIDGains(PIDGains gains) {
        super.pidGains.add(gains.getSlot(), gains);
        
        ClosedLoopSlot slot;
        if (gains.getSlot() == 1) {
            slot = ClosedLoopSlot.kSlot1;
        } else if (gains.getSlot() == 2) {
            slot = ClosedLoopSlot.kSlot2;
        } else if (gains.getSlot() == 3) {
            slot = ClosedLoopSlot.kSlot3;
        } else {
            slot = ClosedLoopSlot.kSlot0;
        }

        this.config.closedLoop
            .p(gains.kP, slot)
            .i(gains.kI, slot)
            .d(gains.kD, slot)
            .feedForward
                .kA(gains.kA, slot)
                .kV(gains.kV, slot)
                .kS(gains.kS, slot)
                .kG(gains.kG, slot)
            ;

        if (gains.softLimit) {
            this.config.softLimit.forwardSoftLimitEnabled(gains.softLimit);
            this.config.softLimit.forwardSoftLimit(gains.softLimitMax);
            this.config.softLimit.reverseSoftLimitEnabled(gains.softLimit);
            this.config.softLimit.reverseSoftLimit(gains.softLimitMax);
        }

        this.motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setPercentOutput(double percent) {
        this.motor.set(percent);
    }

    @Override
    public void setVoltage(double voltage, int slot) {
        this.motor.setVoltage(voltage);
    }

    @Override
    public void setVelocityRPS(double desiredVelocityRPS, int pidSlot) {
        this.motorCtrl.setSetpoint(
            desiredVelocityRPS, 
            ControlType.kMAXMotionVelocityControl, 
            getSlot(pidSlot)
        );
    }

    @Override
    public void setPosition(double desiredRotations, int pidSlot) {
        if (pidGains != null) {
            Utils.clamp(
                desiredRotations, 
                pidGains.get(pidSlot).softLimitMin,
                pidGains.get(pidSlot).softLimitMax
            );
        }
        
        this.motorCtrl.setSetpoint(desiredRotations, ControlType.kMAXMotionPositionControl, getSlot(pidSlot));
    }

    @Override
    public void follow(MotorIO master) {
        this.config.follow(((MotorIOSparkFlex)master).motor, motor.configAccessor.getInverted() != ((MotorIOSparkFlex)master).motor.configAccessor.getInverted());
        this.motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void withFollower(MotorIO follower) {
        follower.follow(this);
    }

    @Override
    public void withStatorCurrentLimit(double currentAmps) {
        this.config.smartCurrentLimit((int)currentAmps);
        this.config.secondaryCurrentLimit(currentAmps);
        this.motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public boolean isInverted() {
        return this.motor.configAccessor.getInverted();
    }

    public void setIdleMode(IdleMode idleMode) {
        com.revrobotics.spark.config.SparkBaseConfig.IdleMode mode;
        switch (idleMode) {
            case kCoast:
                mode = com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast;
                break;
            case kBrake:
            // On default set to brake mode
            default:
            mode = com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake;
                break;
        }

        this.config.idleMode(mode);
        this.motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public double getVelocityRPS() {
        return encoder.getVelocity() / 60d;
    }

    @Override
    public double getRotations() {
        return encoder.getPosition();
    }

    @Override
    public double getAppliedVoltage() {
        return motor.getBusVoltage();
    }

    @Override
    public void resetEncoderPosition(double rotations) {
        this.encoder.setPosition(rotations);
    }

    @Override
    public void setSoftLimits(double min, double max) {
        this.config.softLimit.forwardSoftLimitEnabled(true);
        this.config.softLimit.forwardSoftLimit(max);

        this.config.softLimit.reverseSoftLimitEnabled(true);
        this.config.softLimit.reverseSoftLimit(min);
    }

    @Override
    public void brake() {
        this.motor.stopMotor();
    }

    /**
     * Gets the target PID slot in a useable form
     */
    private ClosedLoopSlot getSlot(int slot) {
        switch (slot) {
            case 1:
                return ClosedLoopSlot.kSlot1;
            case 2:
                return ClosedLoopSlot.kSlot2;
            case 3:
                return ClosedLoopSlot.kSlot3;
            default:
                return ClosedLoopSlot.kSlot0;
        }
    }
}