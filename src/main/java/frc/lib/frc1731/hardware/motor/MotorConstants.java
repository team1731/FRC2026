package frc.lib.frc1731.hardware.motor;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Common motor free-speed and stall constants used by subsystem helpers and mechanism simulation.
 */
public enum MotorConstants {
    kKrakenX60     (6065,  374, 7.16),
    kKrakenX60FOC  (5784,  476, 9.36),
    kKrakenX44     (7757,  279, 4.11),
    kKrakenX44FOC  (7367,  329, 5.01),
    kFalcon500     (6380,  257, 4.69),
    kFalcon500FOC  (6080,  304, 5.84),
    kMinion        (7703,  212, 3.17),
    kVortex        (6825,  391, 5.96),
    kNeo           (5906,  216, 4.20),
    kNeo550        (11000, 100, 0.97),
    ;

    /** Free speed in rotations per second. */
    public final AngularVelocity kMaxVelocity;

    /** Published stall current in amps. */
    public final double kStallCurrent;

    /** Published stall torque in Newton-meters. */
    public final double kStallTorque;

    /** Approximate free-speed motor velocity constant in RPM per volt. */
    public final double kMotorKV;

    private MotorConstants(double maxRPM, double stallAmp, double stallTorque) {
        this.kMaxVelocity = RPM.of(maxRPM);
        this.kStallCurrent = stallAmp;
        this.kStallTorque = stallTorque;
        this.kMotorKV = maxRPM / 12.0;
    }
}
