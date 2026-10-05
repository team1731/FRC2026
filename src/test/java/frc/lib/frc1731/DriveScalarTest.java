package frc.lib.frc1731;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import frc.lib.frc1731.DriveScalar.ScaleType;

class DriveScalarTest {
    @Test
    void maximumSpeedIsAppliedAfterTheCurve() {
        DriveScalar scalar = new DriveScalar(ScaleType.kQuadratic, 0).withScalar(5.12);
        assertEquals(5.12, scalar.scale(1), 1e-9);
        assertEquals(-5.12, scalar.scale(-1), 1e-9);
        assertEquals(1.28, scalar.scale(0.5), 1e-9);
        assertEquals(-1.28, scalar.scale(-0.5), 1e-9);
    }

    @Test
    void deadbandIsSymmetricContinuousAndStillReachesFullSpeed() {
        DriveScalar scalar = new DriveScalar(ScaleType.kLinear, 0.05).withScalar(6);
        assertEquals(0, scalar.scale(0.05));
        assertEquals(0, scalar.scale(-0.05));
        assertEquals(0, scalar.scale(0));
        assertTrue(scalar.scale(0.050001) > 0);
        assertTrue(scalar.scale(0.050001) < 0.00001);
        assertEquals(3, scalar.scale(0.525), 1e-9);
        assertEquals(-3, scalar.scale(-0.525), 1e-9);
        assertEquals(6, scalar.scale(1), 1e-9);
    }

    @Test
    void allCurvesPreserveDirectionAndBoundTheirOutput() {
        for (ScaleType type : ScaleType.values()) {
            DriveScalar scalar = new DriveScalar(type, 0.05).withScalar(5.12);
            double previous = -5.12;
            for (int i = -200; i <= 200; i++) {
                double output = scalar.scale(i / 100.0);
                assertTrue(Math.abs(output) <= 5.12);
                assertTrue(output >= previous);
                assertEquals(-output, scalar.scale(-i / 100.0), 1e-9);
                previous = output;
            }
        }
    }

    @Test
    void rejectsInvalidConfigurationAndInputs() {
        assertThrows(IllegalArgumentException.class, () -> new DriveScalar(ScaleType.kLinear, 1));
        assertThrows(IllegalArgumentException.class, () -> new DriveScalar(ScaleType.kLinear, -0.1));
        assertThrows(IllegalArgumentException.class, () -> new DriveScalar(null, 0));
        DriveScalar scalar = new DriveScalar(ScaleType.kLinear, 0);
        assertThrows(IllegalArgumentException.class, () -> scalar.withScalar(Double.NaN));
        assertThrows(IllegalArgumentException.class, () -> scalar.scale(Double.NaN));
    }
}
