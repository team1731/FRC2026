package frc.lib.frc1731.hardware.motor.io.request;

import static edu.wpi.first.units.Units.*;
import java.util.Objects;
import edu.wpi.first.units.measure.*;
import frc.lib.frc1731.hardware.motor.io.MotorIO;

/** Reusable position request; setters mutate this instance. */
public class PositionRequest extends MotorRequest {
    @Override
    protected double getFeedbackBaseValue(MotorIO io) {
        return io.getPosition().baseUnitMagnitude();
    }

    private Angle epsilonThreshold = Radians.zero();

    /** Sets an absolute, inclusive tolerance; defaults to zero (exact equality). */
    public PositionRequest withEpsilonThreshold(Angle threshold) {
        validateEpsilon(Objects.requireNonNull(threshold).baseUnitMagnitude());
        this.epsilonThreshold = threshold.copy();
        return this;
    }

    public Angle getEpsilonThreshold() { return epsilonThreshold; }

    @Override
    public boolean atSetpoint(MotorIO io) {
        return withinThreshold(getFeedbackBaseValue(io), getBaseValue(),
                epsilonThreshold.baseUnitMagnitude());
    }

    private Angle value = Rotations.zero();

    public PositionRequest() { 
        this(Radians.zero()); 
    }

    public PositionRequest(Angle value) {
        super(RequestType.kPosition);
        withPosition(value);
    }

    public PositionRequest withPosition(Angle value) {
        this.value = Objects.requireNonNull(value).copy();
        return this;
    }

    public Angle getPosition() { 
        return value; 
    }

    @Override
    public PositionRequest withSlot(int slot) { 
        super.withSlot(slot); return this; 
    }

    @Override
    public double getBaseValue() { 
        return value.baseUnitMagnitude(); 
    }

    @Override
    public void apply(MotorIO io) { 
        io.setPosition(value, slot); 
    }
}
