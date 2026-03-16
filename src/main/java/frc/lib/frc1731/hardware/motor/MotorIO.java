package frc.lib.frc1731.hardware.motor;

import java.util.ArrayList;
import java.util.List;

import frc.lib.frc1731.PIDGains;

public abstract class MotorIO {
    protected List<PIDGains> pidGains = new ArrayList<>();
    protected PortConfig portConfig = null;

    protected MotorIO(PortConfig config) {}

    public abstract void follow(MotorIO master);

    public void withFollower(MotorIO follower) {
        follower.follow(this);
    }

    public abstract void withMotionProfile(double velocity, double acceleration);

    public abstract void withPIDGains(PIDGains gains);

    public abstract void setSoftLimits(double min, double max);

    public abstract void setPercentOutput(double percent);

    public abstract void setVelocityRPS(double rps, int pidSlot);

    public void setVelocityRPS(double rps) {
        this.setVelocityRPS(rps, 0);
    }

    public abstract void setPosition(double rotations, int pidSlot);

    public void setPosition(double rotations) {
        this.setPosition(rotations, 0);
    }

    public abstract void setVoltage(double voltage, int slot);

    public void setVoltage(double voltage) {
        this.setVoltage(voltage, 0);
    }

    public abstract double getVelocityRPS();

    public abstract double getRotations();

    public abstract double getAppliedVoltage();

    public abstract void brake();

    public abstract void withStatorCurrentLimit(double amps);

    /**
     * Whether this motor's output is inverted
     */
    public boolean isInverted() {
        return this.portConfig.kInverted;
    }

    public abstract void resetEncoderPosition(double position);
}