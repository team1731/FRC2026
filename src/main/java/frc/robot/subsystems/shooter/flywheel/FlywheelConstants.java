package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.sim.SimpleVelocitySim.SimConstants;

public class FlywheelConstants {
    public static final double kGearRatio = 1d; // 1:1 input:output ratio
    public static final double kMaxVelocity = 100; // Max velocity
    public static final double kWarmupVelocity = 50; // Warmup velocity
    public static final double kEpsilon = 3; // 3 RPS tolerance

    public static final Distance kFlywheelRadius = Inches.of(3.034184).div(2); // 3.034 inch diameter
    public static final Mass kFlywheelMass = Pounds.of(5d); // 5 lb flywheel

    public static final double kCurrentLimit = 70d; // Amps
    public static final double kSupplyCurrentLimit = 30d; // Amps per motor; validate under load.

    public static final SimConstants kSimConstants = new SimConstants(DCMotor.getKrakenX60(4), kGearRatio, kFlywheelRadius, kFlywheelMass);

    public static final PIDGains kVelocityGains = new PIDGains()
        .setP(0.20).setI(0).setD(0).setS(0.15).setV(0.12).setA(0);

    // Optional, uncharacterized VelocityVoltage + FOC tuning baseline (not active).
    // Feedback is motor RPS: kV ~= 12 / (5800 / 60) for an X60 with FOC.
    // P/S are provisional; characterize friction before treating these as tuned gains.
    public static final PIDGains kFOCVoltageGains = new PIDGains()
        .setP(0.20).setI(0).setD(0).setS(0.15).setV(0.124).setA(0);
}
