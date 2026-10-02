package frc.lib.frc1731.hardware.motor.io.request;

import edu.wpi.first.units.measure.*;
import frc.lib.frc1731.hardware.motor.io.MotorIO;

/**
 * Reusable, mutable motor command. Mutators take effect on the next application.
 * Keep one request per independently controlled target; requests are not thread-safe.
 */
public abstract class MotorRequest {
    public final RequestType type;
    public int slot = 0;

    protected MotorRequest(RequestType type) {
        this.type = type;
    }

    public MotorRequest withSlot(int slot) {
        this.slot = slot;
        return this;
    }

    /** Current target in SI units (radians for angular targets, meters for linear targets). */
    public abstract double getBaseValue();

    public abstract void apply(MotorIO io);

    /** Whether feedback is within this request's absolute tolerance. Idle requests return false. */
    public boolean atSetpoint(MotorIO io) { return false; }

    /** Feedback in the same units as getBaseValue; idle requests have no comparable feedback. */
    protected double getFeedbackBaseValue(MotorIO io) { return Double.NaN; }

    /** Strictly above the target, independent of epsilon. Invalid feedback returns false. */
    public boolean isAbove(MotorIO io) {
        double actual = getFeedbackBaseValue(io);
        double target = getBaseValue();
        return Double.isFinite(actual) && Double.isFinite(target) && actual > target;
    }

    /** Strictly above the target, independent of epsilon. Invalid feedback returns false. */
    public boolean isAboveEqual(MotorIO io) {
        double actual = getFeedbackBaseValue(io);
        double target = getBaseValue();
        return Double.isFinite(actual) && Double.isFinite(target) && actual >= target;
    }

    /** Strictly below the target, independent of epsilon. Invalid feedback returns false. */
    public boolean isBelow(MotorIO io) {
        double actual = getFeedbackBaseValue(io);
        double target = getBaseValue();
        return Double.isFinite(actual) && Double.isFinite(target) && actual < target;
    }

    /** Strictly below the target, independent of epsilon. Invalid feedback returns false. */
    public boolean isBelowEqual(MotorIO io) {
        double actual = getFeedbackBaseValue(io);
        double target = getBaseValue();
        return Double.isFinite(actual) && Double.isFinite(target) && actual <= target;
    }

    protected static double validateEpsilon(double epsilon) {
        if (!Double.isFinite(epsilon) || epsilon < 0.0) {
            throw new IllegalArgumentException("Epsilon threshold must be finite and nonnegative");
        }
        return epsilon;
    }

    protected static boolean withinThreshold(double actual, double target, double epsilon) {
        return Double.isFinite(actual) && Double.isFinite(target)
                && Math.abs(actual - target) <= epsilon;
    }

    // Compatibility factories; constructors are preferred for new code.
    public static DutyCycleRequest withDutyCycleRequest(double value) {
        return new DutyCycleRequest(value);
    }

    public static VoltageRequest withVoltageRequest(double value) {
        return new VoltageRequest(value);
    }

    public static PositionRequest withPositionRequest(Angle value) {
        return new PositionRequest(value);
    }

    public static TrapezoidalPositionRequest withTrapezoidalPositionRequest(Angle value) {
        return new TrapezoidalPositionRequest(value);
    }

    public static TrapezoidalPositionRequest withTrapezoidalPositionRequest(
            Angle value, AngularVelocity velocity, AngularAcceleration acceleration) {
        return new TrapezoidalPositionRequest(value, velocity, acceleration);
    }

    public static VelocityRequest withVelocityRequest(AngularVelocity value) {
        return new VelocityRequest(value);
    }

    public static BrakeRequest withBrakeRequest() {
        return new BrakeRequest();
    }

    public static CoastRequest withCoastRequest() {
        return new CoastRequest();
    }

    public static enum RequestType {
        kIdle,
        kDutyCycle,
        kVoltage,
        kVelocity,
        kPosition,
        kPositionTrapezoidal;

        public boolean isIdleRequest() {
            switch (this) {
                case kIdle: return true;
                default: return false;
            }
        }

        public boolean isVoltageRequest() {
            switch (this) {
                case kDutyCycle, kVoltage: return true;
                default: return false;
            }
        }

        public boolean isVelocityRequest() {
            switch (this) {
                case kVelocity: return true;
                default: return false;
            }
        }

        public boolean isPositionRequest() {
            switch (this) {
                case kPosition, kPositionTrapezoidal: return true;
                default: return false;
            }
        }
    }
}
