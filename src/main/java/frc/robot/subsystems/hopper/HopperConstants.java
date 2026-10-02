package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import frc.lib.frc1678.sim.LinearSim;
import frc.lib.frc1678.sim.LinearSim.LinearSimConstants;
import frc.lib.frc1678.util.Util.DistanceAngleConverter;
import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.io.MotorIOSparkMax;
import frc.lib.frc1731.hardware.motor.io.config.SparkMaxIOConfigs;
import frc.robot.Ports;

public final class HopperConstants {
    public static final double kGearing = 20.0; // TODO - Find actual ratio

    public static final int kStatorCurrentLimit = 20;
    public static final int kSupplyCurrentLimit = 20;

    public static final Distance kMaxHeight = Inches.of(7.0);
    public static final Distance kHomeHeight = Inches.of(0);
    public static final Distance kRadius = Inches.of(0.5);
    public static final Distance kEpsilon = Inches.of(0.25);

    public static final PIDGains kPositionGains = new PIDGains().setP(0.5);

    public static final DistanceAngleConverter kConverter = new DistanceAngleConverter(kRadius);

    public static final SparkMaxIOConfigs getIOConfig() {
        return new SparkMaxIOConfigs()
            .withSensorToMechanismRatio(kGearing)
            .withCurrentLimits(kSupplyCurrentLimit, kStatorCurrentLimit)
            .withPIDGains(kPositionGains)
            .withSoftLimits(kConverter.toAngle(kHomeHeight).in(Rotations), kConverter.toAngle(kMaxHeight).in(Rotations))
        ;
    }

    public static final LinearSim getSimulation() {
        LinearSimConstants constants = new LinearSimConstants();
        constants.carriageMass = Pounds.of(3);
        constants.converter = kConverter;
        constants.gearing = kGearing;
        constants.motor = DCMotor.getNeo550(1);
        constants.startingHeight = kHomeHeight;
        constants.minHeight = kHomeHeight;
        constants.maxHeight = kMaxHeight;
        return new LinearSim(constants);
    }

    public static final MotorIOSparkMax getIO() {
        return MotorIOSparkMax.generateNeo550(Ports.kSqueezerConfig, getIOConfig()).withSimulation(getSimulation());
    }
}
