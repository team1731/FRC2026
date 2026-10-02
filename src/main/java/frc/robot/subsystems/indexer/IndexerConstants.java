package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import frc.lib.frc1678.sim.RollerSim;
import frc.lib.frc1678.sim.RollerSim.RollerSimConstants;
import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.io.MotorIOTalonFX;
import frc.lib.frc1731.hardware.motor.io.config.TalonFXIOConfigs;
import frc.robot.Ports;

public final class IndexerConstants {
    public static final double kGearRatio = 3.0;

    public static final AngularVelocity kFeedVelocity = RotationsPerSecond.of(60);
    public static final AngularVelocity kSpitVelocity = RotationsPerSecond.of(-60);
    public static final AngularVelocity kEpsilon = RotationsPerSecond.one();

    public static final double kSupplyCurrentLimit = 40.0;
    public static final double kStatorCurrentLimit = 120.0;

    public static final Distance kRollerDiameter = Inches.of(1.398);

    public static final PIDGains kVelocityGains = new PIDGains()
        .setP(0.10)
        .setS(0.15)
        .setV(0.12);
    
    public static final TalonFXIOConfigs getIOConfig() {
        return new TalonFXIOConfigs()
            .withPIDGains(kVelocityGains)
            .withSensorToMechanismRatio(kGearRatio)
            .withNeutralMode(NeutralModeValue.Coast)
            .withFollower(Ports.kTopKickerConfig.kPort)
            .withCurrentLimits(kSupplyCurrentLimit, kStatorCurrentLimit)
            .invert()
            .brake()
        ;
    }

    public static final RollerSim getSimulation() {
        return new RollerSim(
            new RollerSimConstants()
            .withGearing(kGearRatio)
            .withMOI(0.003)
            .withMotor(DCMotor.getKrakenX60(1))
        );
    }

    public static final MotorIOTalonFX getIO() {
        return MotorIOTalonFX.generateKrakenX60FOC(Ports.kIndexerFloorConfig, getIOConfig()).withSimulation(getSimulation());
    }
}