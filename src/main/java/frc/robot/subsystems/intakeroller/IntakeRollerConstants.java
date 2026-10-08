package frc.robot.subsystems.intakeroller;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.frc1678.sim.MechanismSim;
import frc.lib.frc1678.sim.RollerSim;
import frc.lib.frc1678.sim.RollerSim.RollerSimConstants;
import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.MotorConstants;
import frc.lib.frc1731.hardware.motor.io.MotorIOTalonFX;
import frc.lib.frc1731.hardware.motor.io.config.TalonFXIOConfigs;
import frc.robot.Ports;

public class IntakeRollerConstants {
    public static final double kGearRatio = 3.0;

    public static final PIDGains kPIDGains = new PIDGains()
        .setP(0.15)
        .setV(0.30);

    public static final AngularVelocity kIntakeVelocity = MotorConstants.kKrakenX44FOC.kMaxVelocity.div(kGearRatio);

    public static final double kStatorCurrentLimit = 120;
    public static final double kSupplyCurrentLimit = 60;

    public static final TalonFXIOConfigs getIOConfig() {
        return new TalonFXIOConfigs()
            .withPIDGains(kPIDGains)
            .brake();
    }

    public static final MechanismSim getSim() {
        return new RollerSim(
            new RollerSimConstants()
                .withGearing(kGearRatio)
                .withMotor(DCMotor.getKrakenX44(1))
                .withMOI(0.001)
        );
    }

    public static final MotorIOTalonFX getIO() {
        return MotorIOTalonFX.generateKrakenX44(Ports.kIntakeRollerConfig, getIOConfig()).withSimulation(getSim());
    }

    @AutoLog
    public static class IntakeRollerIOInputs {
        public double currentVelocity, setpointVelocity;
    }
}