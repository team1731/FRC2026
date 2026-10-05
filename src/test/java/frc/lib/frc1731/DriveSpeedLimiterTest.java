package frc.lib.frc1731;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

class DriveSpeedLimiterTest {
    @Test
    void preservesFeasibleRequestsAndZeroInput() {
        var result = DriveSpeedLimiter.prioritizeRotation(1, -1, 2, 5.12, 0.4);
        assertEquals(1, result.vxMetersPerSecond);
        assertEquals(-1, result.vyMetersPerSecond);
        assertEquals(2, result.omegaRadiansPerSecond);
        var zero = DriveSpeedLimiter.prioritizeRotation(0, 0, 0, 5.12, 0.4);
        assertEquals(0, zero.vxMetersPerSecond);
        assertEquals(0, zero.vyMetersPerSecond);
        assertEquals(0, zero.omegaRadiansPerSecond);
    }

    @Test
    void fullSpeedTurnsRetainRotationAndTranslationDirection() {
        var result = DriveSpeedLimiter.prioritizeRotation(5.12, 5.12, -5.65, 5.12, 0.4);
        assertEquals(-5.65, result.omegaRadiansPerSecond);
        assertEquals(2.86, Math.hypot(result.vxMetersPerSecond, result.vyMetersPerSecond), 1e-9);
        assertEquals(result.vxMetersPerSecond, result.vyMetersPerSecond);
    }

    @Test
    void rotationAloneCannotExceedWheelSpeed() {
        var result = DriveSpeedLimiter.prioritizeRotation(5, 0, 100, 5, 0.4);
        assertEquals(12.5, result.omegaRadiansPerSecond);
        assertEquals(0, result.vxMetersPerSecond);
        assertEquals(0, result.vyMetersPerSecond);
    }

    @Test
    void everyModuleStaysWithinItsSpeedLimitAcrossDirectionsAndTurnRates() {
        double x = 0.257175;
        double y = 0.295275;
        double radius = Math.hypot(x, y);
        for (int degrees = 0; degrees < 360; degrees += 15) {
            double angle = Math.toRadians(degrees);
            for (int omega = -15; omega <= 15; omega++) {
                var result = DriveSpeedLimiter.prioritizeRotation(
                    7 * Math.cos(angle), 7 * Math.sin(angle), omega, 5.12, radius);
                for (double moduleX : new double[] {-x, x}) {
                    for (double moduleY : new double[] {-y, y}) {
                        double moduleSpeed = Math.hypot(
                            result.vxMetersPerSecond - result.omegaRadiansPerSecond * moduleY,
                            result.vyMetersPerSecond + result.omegaRadiansPerSecond * moduleX);
                        assertTrue(moduleSpeed <= 5.12 + 1e-9);
                    }
                }
            }
        }
    }
}
