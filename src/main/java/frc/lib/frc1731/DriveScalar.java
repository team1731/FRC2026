package frc.lib.frc1731;

import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * Applies deadband, input shaping, and optional slew limiting to joystick drive inputs.
 */
public class DriveScalar {
    private double m_deadband = 0.0;
    private ScaleType m_type = ScaleType.kLinear;
    private SlewRateLimiter m_limiter = null;

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
        this.m_deadband = deadband;
        this.m_type = scaleType;
    }

    /**
     * Adds a smoothing factor to joystick inputs 
     * @param limiter configuration for joystick limiting
     */
    public DriveScalar withSlewLimiter(SlewRateLimiter limiter) {
        this.m_limiter = limiter;
        return this;
    }

    /**
     * Applies the configured deadband, response curve, and optional slew limiter.
     *
     * @param input raw input value
     * @return shaped output value
     */
    public double scale(double input) {
        // Zero out based on magnitude, not raw value, so negative inputs
        // within the deadband are also zeroed.
        double output = Math.abs(input) < m_deadband ? 0.0 : input;

        // Apply the power curve to magnitude, then restore sign, so
        // even-power curves (quadratic) don't flip negative inputs positive.
        double power = m_type.ordinal() + 1; // kLinear=1, kQuadratic=2, kCubic=3
        output = Math.copySign(Math.pow(Math.abs(output), power), output);

        if (m_limiter != null) {
            output = m_limiter.calculate(output);
        }

        return output;
    }
}
