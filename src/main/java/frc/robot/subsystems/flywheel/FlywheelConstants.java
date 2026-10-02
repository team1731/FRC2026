package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.lib.frc1678.sim.RollerSim;
import frc.lib.frc1678.sim.RollerSim.RollerSimConstants;
import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.io.MotorIOTalonFX;
import frc.lib.frc1731.hardware.motor.io.config.TalonFXIOConfigs;
import frc.robot.Ports;

public final class FlywheelConstants {
    public static final double kGearRatio = 1.0; // direct 1:1 powering of flywheel
    public static final double kStatorCurrentLimit = 70.0;
    public static final double kSupplyCurrentLimit = 30.0;

    public static final AngularVelocity kWarmupVelocity = RotationsPerSecond.of(50);
    public static final AngularVelocity kEpsilon = RotationsPerSecond.of(3.0);

    public static final Distance kFlywheelRadius = Inches.of(3.034184).div(2); // 3.034 inch diameter
    public static final Mass kFlywheelMass = Pounds.of(5d); // 5 lb flywheel

    public static final MomentOfInertia kFlywheelMOI =
        KilogramSquareMeters.of(
            0.5 * kFlywheelMass.in(Kilograms)
                * Math.pow(kFlywheelRadius.in(Meters), 2)
        );

    public static final PIDGains kShotGains = new PIDGains()
        .setP(0.20).setS(0.15).setV(0.12);

    public static final TalonFXIOConfigs getIOConfig() {
        return new TalonFXIOConfigs()
            .withCurrentLimits(kSupplyCurrentLimit, kStatorCurrentLimit)
            .withPIDGains(kShotGains)
            .withFollower(Ports.kLeftFlywheelBottomConfig.kPort, false)
            .withFollower(Ports.kRightFlywheelBottomConfig.kPort, true)
            .withFollower(Ports.kRightFlywheelTopConfig.kPort, true)
            .coast()
        ;
    }

    public static final RollerSim getSim() {
        return new RollerSim(
            new RollerSimConstants()
                .withGearing(kGearRatio)
                .withMOI(kFlywheelMOI.in(KilogramSquareMeters))
                .withMotor(DCMotor.getKrakenX60(4))
        );
    }

    public static final MotorIOTalonFX getIO() {
        return MotorIOTalonFX.generateKrakenX60(Ports.kLeftFlywheelTopMasterConfig, getIOConfig()).withSimulation(getSim());
    }
}