package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.PortConfig;
import frc.lib.frc1731.sim.SimpleVelocitySim.SimConstants;

public class FlywheelConstants {
    public static final double kGearRatio = 1d; // 1:1 input:output ratio
    public static final AngularVelocity kMaxVelocity = RotationsPerSecond.of(100);
    public static final AngularVelocity kStallVelocity = RotationsPerSecond.of(20);
    public static final AngularVelocity kEpsilon = RotationsPerSecond.one(); // 1 RPS tolerance

    public static final Distance kFlywheelRadius = Inches.of(2d); // 4 inch diameter
    public static final Mass kFlywheelMass = Pounds.of(1d); // 1 lb flywheel

    public static final PortConfig kLeftFlywheelConfig = new PortConfig("rio", 0, false);
    public static final PortConfig kRightFlywheelConfig = new PortConfig("rio", 0, false);

    public static final double kCurrentLimit = 60d; // Amps

    public static final SimConstants kSimConstants = new SimConstants(DCMotor.getKrakenX60(1), kGearRatio, kFlywheelRadius, kFlywheelMass);

    public static final PIDGains kVelocityGains = new PIDGains() // Tuned sys-id via simulation
        .setP(0.040022)
        .setA(0.0059818)
        .setV(0.11934)
        .setS(0.012762)
        ;
}