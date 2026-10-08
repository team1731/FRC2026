package frc.robot.subsystems.kicker;


import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.frc1678.sim.RollerSim;
import frc.lib.frc1678.sim.RollerSim.RollerSimConstants;
import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.io.MotorIOTalonFX;
import frc.lib.frc1731.hardware.motor.io.config.TalonFXIOConfigs;
import frc.robot.Ports;

public final class KickerConstants {
    public static final double kGearRatio = 32.0 / 60.0; // 60t input, 32t output

    public static final AngularVelocity kFeedVelocity = RotationsPerSecond.of(65);
    public static final AngularVelocity kSpitVelocity = RotationsPerSecond.of(-50);
    public static final AngularVelocity kEpsilon = RotationsPerSecond.one();

    public static final double kStatorCurrentLimit = 120.0;
    public static final double kSupplyCurrentLimit = 50.0;

    public static final PIDGains kVelocityGains = new PIDGains()
        .setP(0.15)
        .setS(0.15)
        .setV(0.12);
    
    public static final TalonFXIOConfigs getIOConfig() {
        return new TalonFXIOConfigs()
            .withPIDGains(kVelocityGains)
            .withSensorToMechanismRatio(kGearRatio)
            .withNeutralMode(NeutralModeValue.Coast)
            .withFollower(Ports.kTopKickerConfig.kPort)
            .brake()
        ;
    }

    public static final RollerSim getSimulation() {
        return new RollerSim(
            new RollerSimConstants()
            .withGearing(kGearRatio)
            .withMOI(0.003)
            .withMotor(DCMotor.getKrakenX60(2))
        );
    }

    public static final MotorIOTalonFX getIO() {
        return MotorIOTalonFX.generateKrakenX60FOC(Ports.kBottomKickerMasterConfig, getIOConfig()).withSimulation(getSimulation());
    }

    @AutoLog
    public static class KickerIOInputs {
        public double currentVelocity, setpointVelocity;
    }
}
