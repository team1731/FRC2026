package frc.lib.frc1731;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Reserves wheel speed for rotation by reducing translation without changing its direction. */
public final class DriveSpeedLimiter {
    private DriveSpeedLimiter() {}

    /**
     * Applies the conservative bound |translation| + |omega| * radius <= maxWheelSpeed.
     * This works in robot or field coordinates and for either alliance perspective.
     */
    public static ChassisSpeeds prioritizeRotation(
            double vx, double vy, double omega, double maxWheelSpeed, double driveRadius) {
        if (!Double.isFinite(maxWheelSpeed) || maxWheelSpeed <= 0
                || !Double.isFinite(driveRadius) || driveRadius <= 0
                || !Double.isFinite(vx) || !Double.isFinite(vy) || !Double.isFinite(omega)) {
            throw new IllegalArgumentException("Finite speeds and positive wheel speed/radius required");
        }
        double limitedOmega = MathUtil.clamp(omega, -maxWheelSpeed / driveRadius, maxWheelSpeed / driveRadius);
        double translationLimit = Math.max(0, maxWheelSpeed - Math.abs(limitedOmega) * driveRadius);
        double magnitude = Math.hypot(vx, vy);
        double scale = magnitude > translationLimit ? translationLimit / magnitude : 1.0;
        return new ChassisSpeeds(vx * scale, vy * scale, limitedOmega);
    }
}
