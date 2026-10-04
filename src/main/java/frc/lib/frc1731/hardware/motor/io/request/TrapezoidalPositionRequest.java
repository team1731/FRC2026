package frc.lib.frc1731.hardware.motor.io.request;

import static edu.wpi.first.units.Units.*;
import java.util.Objects;

import edu.wpi.first.units.measure.*;
import frc.lib.frc1731.hardware.motor.io.MotorIO;

/**
 * Reusable profiled position request. Without explicit constraints, uses the controller's current constraints.
 */
public class TrapezoidalPositionRequest extends MotorRequest {
    @Override
    protected double getFeedbackBaseValue(MotorIO io) {
        return io.getPosition().baseUnitMagnitude();
    }

    private Angle epsilonThreshold = Radians.zero();

    /** Sets an absolute, inclusive tolerance; defaults to zero (exact equality). */
    public TrapezoidalPositionRequest withEpsilonThreshold(Angle threshold) {
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

    private Angle position = Rotations.zero();
    private AngularVelocity maxVelocity = RPM.zero();
    private AngularAcceleration maxAcceleration = RPM.per(Second).zero();

    public TrapezoidalPositionRequest() {
        this(Radians.zero());
    }

    public TrapezoidalPositionRequest(Angle position) {
        super(RequestType.kPositionTrapezoidal);
        withPosition(position);
    }

    public TrapezoidalPositionRequest(AngularVelocity maxVelocity,
            AngularAcceleration maxAcceleration) {
        this(Radians.zero(), maxVelocity, maxAcceleration);
    }

    public TrapezoidalPositionRequest(Angle position, AngularVelocity maxVelocity,
            AngularAcceleration maxAcceleration) {
        this(position);
        withSpeeds(maxVelocity, maxAcceleration);
    }

    public TrapezoidalPositionRequest withPosition(Angle position) {
        this.position = Objects.requireNonNull(position).copy();
        return this;
    }

    public TrapezoidalPositionRequest withDegrees(double degrees) {
        this.position = Degrees.of(Objects.requireNonNull(degrees)).copy();
        return this;
    }

    public TrapezoidalPositionRequest withRotations(double rotations) {
        this.position = Rotations.of(Objects.requireNonNull(rotations)).copy();
        return this;
    }

    public TrapezoidalPositionRequest withSpeeds(AngularVelocity maxVelocity, AngularAcceleration maxAcceleration) {
        Objects.requireNonNull(maxVelocity);
        Objects.requireNonNull(maxAcceleration);
        this.maxVelocity = maxVelocity.copy();
        this.maxAcceleration = maxAcceleration.copy();
        return this;
    }

    public Angle getPosition() { return position; }

    @Override
    public TrapezoidalPositionRequest withSlot(int slot) { super.withSlot(slot); return this; }

    @Override
    public double getBaseValue() { return position.baseUnitMagnitude(); }

    @Override
    public void apply(MotorIO io) {
        if (maxVelocity != null && maxAcceleration != null) {
            io.updateTrapezoidalSpeeds(maxVelocity, maxAcceleration, slot);
        }
        io.setPositionTrapezoidal(position, slot);
    }
}
