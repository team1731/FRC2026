package frc.lib.frc1731;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.MathUtil;

/**
 * Applies deadband, input shaping, and optional slew limiting to joystick drive inputs.
 */
public class DriveScalar {
    private double m_deadband = 0.0;
    private ScaleType m_type = ScaleType.kLinear;
    private SlewRateLimiter m_limiter = null;
    private double m_scalar = 1.0;

    /** Available joystick response curves. */
    public static enum ScaleType {
        /** Leaves input magnitude linear after deadband. */
        kLinear,

        /** Squares input magnitude while preserving sign. */
        kQuadratic,

        /** Cubes input magnitude while preserving sign. */
        kCubic
    }

    /**
     * Creates a drive input scaler.
     *
     * @param scaleType response curve to apply after deadband
     * @param deadband input magnitude below which output should be zero
     */
    public DriveScalar(ScaleType scaleType, double deadband) {
        if (scaleType == null || !Double.isFinite(deadband) || deadband < 0 || deadband >= 1) {
            throw new IllegalArgumentException("A curve and a deadband in [0, 1) are required");
        }
        this.m_deadband = deadband;
        this.m_type = scaleType;
    }

    /** Sets the output multiplier (for example, maximum speed in m/s or rad/s). */
    public DriveScalar withScalar(double scalar) {
        if (!Double.isFinite(scalar) || scalar < 0) {
            throw new IllegalArgumentException("Scalar must be finite and nonnegative");
        }
        m_scalar = scalar;
        return this;
    }

    /**
     * Adds a smoothing factor to joystick inputs 
     * @param limiter limiter in normalized joystick units per second, before output scaling
     */
    public DriveScalar withSlewLimiter(SlewRateLimiter limiter) {
        this.m_limiter = limiter;
        return this;
    }

    /**
     * Applies the configured deadband, response curve, and optional slew limiter.
     *
     * @param input normalized joystick input in [-1, 1], clamped if outside that range
     * @return shaped input multiplied by the configured output scalar
     */
    public double scale(double input) {
        if (!Double.isFinite(input)) {
            throw new IllegalArgumentException("Joystick input must be finite");
        }
        double output = MathUtil.applyDeadband(MathUtil.clamp(input, -1.0, 1.0), m_deadband);

        // Apply the power curve to magnitude, then restore sign, so
        // even-power curves (quadratic) don't flip negative inputs positive.
        double power = m_type.ordinal() + 1; // kLinear=1, kQuadratic=2, kCubic=3
        output = Math.copySign(Math.pow(Math.abs(output), power), output);

        if (m_limiter != null) {
            output = m_limiter.calculate(output);
        }

        return output * m_scalar;
    }
}
