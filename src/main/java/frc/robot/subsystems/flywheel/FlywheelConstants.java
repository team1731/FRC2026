package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.PortConfig;

public class FlywheelConstants {
    public static final double kGearRatio = 1d; // 1:1 input:output ratio
    public static final double kMaxVelocityRPS = 100d;
    public static final double kStallVelocityRPS = 20d;
    public static final Distance kFlywheelRadius = Inches.of(2d); // 4 inch diameter
    public static final Mass kFlywheelMass = Pounds.of(1d); // 1 lb flywheel
    public static final double kEpsilon = 1d; // 1 RPS tolerance

    public static final PortConfig kLeftFlywheelConfig = new PortConfig("rio", 0, false);
    public static final PortConfig kRightFlywheelConfig = new PortConfig("rio", 0, false);

    // public static final PIDGains kVelocityGains = new PIDGains()
    //     .setP(0.01) // 1 RPS Error = 0.01 V (Max error gain = 1 V)
    //     .setV(11.75 / kMaxVelocityRPS) // 11 V for 100 RPS Velocity
    //     .setS(0.1) // Add 0.1 V to overcome static friction
    //     ;

    public static final PIDGains kVelocityGains = new PIDGains()
        .setP(0.040022)
        .setA(0.0059818)
        .setV(0.11934)
        .setS(0.012762)
        ;
}