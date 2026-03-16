package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.sim.SimpleVelocitySim.SimConstants;
import frc.robot.Ports;

public class FlywheelConstants {
    public static final double kGearRatio = 1d; // 1:1 input:output ratio
    public static final double kMaxVelocity = 100; // Max velocity
    public static final double kWarmupVelocity = 40; // Warmup velocity
    public static final double kEpsilon = 2; // 2 RPS tolerance

    public static final Distance kFlywheelRadius = Inches.of(2d); // 4 inch diameter
    public static final Mass kFlywheelMass = Pounds.of(1d); // 1 lb flywheel

    public static final FlywheelConfiguration kLeftFlywheelConfig = new FlywheelConfiguration("Left", Ports.kLeftFlywheelConfig);
    public static final FlywheelConfiguration kRightFlywheelConfig = new FlywheelConfiguration("Right", Ports.kRightFlywheelConfig);

    public static final double kCurrentLimit = 60d; // Amps

    public static final SimConstants kSimConstants = new SimConstants(DCMotor.getKrakenX60(1), kGearRatio, kFlywheelRadius, kFlywheelMass);

    public static final PIDGains kVelocityGains = new PIDGains()
        .setP(0.1)
        .setV(0.15)
        ;
}