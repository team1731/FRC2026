package frc.lib.frc1731.hardware.motor.io.request;

import static edu.wpi.first.units.Units.*;
import java.util.Objects;
import edu.wpi.first.units.measure.*;
import frc.lib.frc1731.hardware.motor.io.MotorIO;

/** Reusable velocity request; setters mutate this instance. */
public class VelocityRequest extends MotorRequest {
    @Override
    protected double getFeedbackBaseValue(MotorIO io) {
        return io.getVelocity().baseUnitMagnitude();
    }

    private AngularVelocity epsilonThreshold = RadiansPerSecond.zero();

    /** Sets an absolute, inclusive tolerance; defaults to zero (exact equality). */
    public VelocityRequest withEpsilonThreshold(AngularVelocity threshold) {
        validateEpsilon(Objects.requireNonNull(threshold).baseUnitMagnitude());
        this.epsilonThreshold = threshold.copy();
        return this;
    }

    public AngularVelocity getEpsilonThreshold() { return epsilonThreshold; }

    @Override
    public boolean atSetpoint(MotorIO io) {
        return withinThreshold(getFeedbackBaseValue(io), getBaseValue(),
                epsilonThreshold.baseUnitMagnitude());
    }

    private AngularVelocity setpoint = RPM.zero();

    public VelocityRequest() { 
        this(RadiansPerSecond.zero()); 
    }

    public VelocityRequest(AngularVelocity setpoint) {
        super(RequestType.kVelocity);
        withVelocity(setpoint);
    }

    /** Updates the target in rotations per minute and returns this request. */
    public VelocityRequest withRPM(double rpm) {
        return withVelocity(RPM.of(rpm));
    }

    /** Updates the target in rotations per second and returns this request. */
    public VelocityRequest withRPS(double rps) {
        return withVelocity(RotationsPerSecond.of(rps));
    }

    public VelocityRequest withVelocity(AngularVelocity setpoint) {
        this.setpoint = Objects.requireNonNull(setpoint).copy();
        return this;
    }

    public AngularVelocity getVelocity() { 
        return setpoint; 
    }

    @Override
    public VelocityRequest withSlot(int slot) { 
        super.withSlot(slot); return this; 
    }

    @Override
    public double getBaseValue() { 
        return setpoint.baseUnitMagnitude(); 
    }

    @Override
    public void apply(MotorIO io) { 
        io.setVelocity(setpoint, slot); 
    }
}
