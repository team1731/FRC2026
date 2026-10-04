package frc.lib.frc1731.hardware.motor.io.request;

import static edu.wpi.first.units.Units.*;
import java.util.Objects;
import edu.wpi.first.units.measure.*;
import frc.lib.frc1731.hardware.motor.io.MotorIO;

/** Reusable voltage request; setters mutate this instance. */
public class VoltageRequest extends MotorRequest {
    @Override
    protected double getFeedbackBaseValue(MotorIO io) {
        return io.getAppliedVoltage();
    }

    private Voltage epsilonThreshold = Volts.zero();

    /** Sets an absolute, inclusive tolerance; defaults to zero (exact equality). */
    public VoltageRequest withEpsilonThreshold(Voltage threshold) {
        validateEpsilon(Objects.requireNonNull(threshold).baseUnitMagnitude());
        this.epsilonThreshold = threshold.copy();
        return this;
    }

    public Voltage getEpsilonThreshold() { return epsilonThreshold; }

    @Override
    public boolean atSetpoint(MotorIO io) {
        return withinThreshold(getFeedbackBaseValue(io), getBaseValue(),
                epsilonThreshold.baseUnitMagnitude());
    }

    private Voltage setpoint = Volts.zero();

    public VoltageRequest() { 
        this(Volts.zero()); 
    }

    public VoltageRequest(Voltage setpoint) {
        super(RequestType.kVoltage);
        withVoltage(setpoint);
    }

    public VoltageRequest(double setpoint) { 
        this(Volts.of(setpoint)); 
    }

    public VoltageRequest withVoltage(double setpoint) { 
        return withVoltage(Volts.of(setpoint)); 
    }

    public VoltageRequest withVoltage(Voltage setpoint) {
        this.setpoint = Objects.requireNonNull(setpoint).copy();
        return this;
    }

    public Voltage getVoltage() { 
        return setpoint; 
    }

    @Override
    public VoltageRequest withSlot(int slot) { 
        super.withSlot(slot); 
        return this; 
    }

    @Override
    public double getBaseValue() { 
        return setpoint.baseUnitMagnitude(); 
    }

    @Override
    public void apply(MotorIO io) { 
        io.setVoltage(setpoint.in(Volts)); 
    }
}
