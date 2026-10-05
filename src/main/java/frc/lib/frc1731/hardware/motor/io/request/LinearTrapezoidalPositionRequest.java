package frc.lib.frc1731.hardware.motor.io.request;

import static edu.wpi.first.units.Units.*;
import java.util.Objects;
import edu.wpi.first.units.measure.*;
import frc.lib.frc1731.hardware.motor.io.MotorIO;
import frc.lib.frc1678.util.Util.DistanceAngleConverter;

/**
 * Reusable profiled position request. Without explicit constraints, uses the controller's current constraints.
 */
public class LinearTrapezoidalPositionRequest extends MotorRequest {
    @Override
    protected double getFeedbackBaseValue(MotorIO io) {
        return converter.toDistance(io.getPosition()).baseUnitMagnitude();
    }

    private Distance epsilonThreshold = Meters.zero();

    /** Sets an absolute, inclusive tolerance; defaults to zero (exact equality). */
    public LinearTrapezoidalPositionRequest withEpsilonThreshold(Distance threshold) {
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
    private LinearVelocity maxVelocity;
    private LinearAcceleration maxAcceleration;
    private final DistanceAngleConverter converter;

    public LinearTrapezoidalPositionRequest(DistanceAngleConverter converter) {
        this(Meters.zero(), converter);
    }

    public LinearTrapezoidalPositionRequest(Distance position, DistanceAngleConverter converter) {
        super(RequestType.kPositionTrapezoidal);
        this.converter = Objects.requireNonNull(converter);
        withPosition(position);
    }

    public LinearTrapezoidalPositionRequest(DistanceAngleConverter converter, LinearVelocity maxVelocity, LinearAcceleration maxAcceleration) {
        super(RequestType.kPositionTrapezoidal);
        this.converter = Objects.requireNonNull(converter);
        withSpeeds(maxVelocity, maxAcceleration);
    }

    public LinearTrapezoidalPositionRequest(Distance position, LinearVelocity maxVelocity,
            LinearAcceleration maxAcceleration, DistanceAngleConverter converter) {
        this(position, converter);
        withSpeeds(maxVelocity, maxAcceleration);
    }

    /** Updates the target in meters and returns this request. */
    public LinearTrapezoidalPositionRequest withMeters(double meters) {
        return withPosition(Meters.of(meters));
    }

    /** Updates the target in inches and returns this request. */
    public LinearTrapezoidalPositionRequest withInches(double inches) {
        return withPosition(Inches.of(inches));
    }

    public LinearTrapezoidalPositionRequest withPosition(Distance position) {
        this.position = Objects.requireNonNull(position).copy();
        return this;
    }

    public LinearTrapezoidalPositionRequest withSpeeds(LinearVelocity maxVelocity, LinearAcceleration maxAcceleration) {
        Objects.requireNonNull(maxVelocity);
        Objects.requireNonNull(maxAcceleration);
        this.maxVelocity = maxVelocity.copy();
        this.maxAcceleration = maxAcceleration.copy();
        return this;
    }

    public Distance getPosition() { return position; }

    @Override
    public LinearTrapezoidalPositionRequest withSlot(int slot) { super.withSlot(slot); return this; }

    @Override
    public double getBaseValue() { return position.baseUnitMagnitude(); }

    @Override
    public void apply(MotorIO io) {
        if (maxVelocity != null && maxAcceleration != null) {
            io.updateTrapezoidalSpeeds(converter.toAngularVelocity(maxVelocity),
                    RadiansPerSecondPerSecond.of(maxAcceleration.in(MetersPerSecondPerSecond)
                            / converter.getDrumRadius().in(Meters)), slot);
        }
        io.setPositionTrapezoidal(converter.toAngle(position), slot);
    }
}
