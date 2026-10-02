package frc.lib.frc1731.hardware.motor.io.request;

import frc.lib.frc1731.hardware.motor.io.MotorIO;

/** Reusable dutycycle request; setters mutate this instance. */
public class DutyCycleRequest extends MotorRequest {
    @Override
    protected double getFeedbackBaseValue(MotorIO io) {
        return io.getAppliedDutyCycle();
    }

    private double epsilonThreshold = 0.0;

    /** Sets an absolute, inclusive tolerance; defaults to zero (exact equality). */
    public DutyCycleRequest withEpsilonThreshold(double threshold) {
        validateEpsilon(threshold);
        this.epsilonThreshold = threshold;
        return this;
    }

    public double getEpsilonThreshold() { return epsilonThreshold; }

    @Override
    public boolean atSetpoint(MotorIO io) {
        return withinThreshold(getFeedbackBaseValue(io), getBaseValue(),
                epsilonThreshold);
    }

    private double setpoint;

    public DutyCycleRequest() {
        this(0.0);
    }

    public DutyCycleRequest(double setpoint) {
        super(RequestType.kDutyCycle);
        withPercent(setpoint);
    }

    public DutyCycleRequest withPercent(double setpoint) {
        this.setpoint = setpoint;
        return this;
    }

    @Override
    public DutyCycleRequest withSlot(int slot) { 
        super.withSlot(slot); return this; 
    }

    @Override
    public double getBaseValue() { 
        return setpoint; 
    }

    @Override
    public void apply(MotorIO io) { 
        io.setPercent(setpoint); 
    }
}
