package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import frc.lib.frc1678.sim.RollerSim.RollerSimConstants;
import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.PortConfig;

public class FlywheelConstants {
    public static final double kGearRatio = 1d; // 1:1 input:output ratio
    public static final double kMaxVelocityRPS = 100d;
    public static final Distance kFlywheelRadius = Inches.of(2d); // 4 inch diameter
    public static final double kMOI = Math.pow(kFlywheelRadius.in(Meters), 2) * Pounds.of(1d).div(2d).in(Kilograms); // I = 1/2 * MR^2
    public static final double kEpsilon = 1d; // 1 RPS tolerance

    public static final PortConfig kLeftFlywheelConfig = new PortConfig("rio", 0, false);
    public static final PortConfig kRightFlywheelConfig = new PortConfig("rio", 0, false);
    public static final PIDGains kVelocityGains = new PIDGains()
        .setP(0.01) // 1 RPS Error = 0.01 V
        .setV(11.0 / kMaxVelocityRPS) // 12 V for 100 RPS Velocity
        .setS(0.1) // Add 0.1 V to overcome static friction
        ;

    public static final RollerSimConstants kRollerSimConstants = new RollerSimConstants(DCMotor.getKrakenX60(1), kGearRatio, kMOI);
}