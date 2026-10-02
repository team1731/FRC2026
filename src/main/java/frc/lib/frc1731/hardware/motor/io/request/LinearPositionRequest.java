package frc.lib.frc1731.hardware.motor.io.request;

import static edu.wpi.first.units.Units.*;
import java.util.Objects;
import edu.wpi.first.units.measure.*;
import frc.lib.frc1731.hardware.motor.io.MotorIO;
import frc.lib.frc1678.util.Util.DistanceAngleConverter;

/** Reusable linear target, converted to a mechanism angle when applied. */
public class LinearPositionRequest extends MotorRequest {
    @Override
    protected double getFeedbackBaseValue(MotorIO io) {
        return converter.toDistance(io.getPosition()).baseUnitMagnitude();
    }

    private Distance epsilonThreshold = Meters.zero();

    /** Sets an absolute, inclusive tolerance; defaults to zero (exact equality). */
    public LinearPositionRequest withEpsilonThreshold(Distance threshold) {
        validateEpsilon(Objects.requireNonNull(threshold).baseUnitMagnitude());
        this.epsilonThreshold = threshold.copy();
        return this;
    }

    public Distance getEpsilonThreshold() { return epsilonThreshold; }

    @Override
    public boolean atSetpoint(MotorIO io) {
        return withinThreshold(getFeedbackBaseValue(io), getBaseValue(),
                epsilonThreshold.baseUnitMagnitude());
    }

    private Distance position = Inches.zero();
    private final DistanceAngleConverter converter;

    public LinearPositionRequest(DistanceAngleConverter converter) { 
        this(Meters.zero(), converter); 
    }

    public LinearPositionRequest(Distance position, DistanceAngleConverter converter) {
        super(RequestType.kPosition);
        this.converter = Objects.requireNonNull(converter);
        withPosition(position);
    }

    /** Updates the target in meters and returns this request. */
    public LinearPositionRequest withMeters(double meters) {
        return withPosition(Meters.of(meters));
    }

    /** Updates the target in inches and returns this request. */
    public LinearPositionRequest withInches(double inches) {
        return withPosition(Inches.of(inches));
    }

    public LinearPositionRequest withPosition(Distance position) {
        this.position = Objects.requireNonNull(position).copy();
        return this;
    }

    public Distance getPosition() { 
        return position; 
    }

    @Override
    public LinearPositionRequest withSlot(int slot) { 
        super.withSlot(slot); return this; 
    }

    @Override
    public double getBaseValue() { 
        return position.baseUnitMagnitude(); 
    }

    @Override
    public void apply(MotorIO io) { 
        io.setPosition(converter.toAngle(position), slot); 
    }
}
