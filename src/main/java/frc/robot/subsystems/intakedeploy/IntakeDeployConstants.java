package frc.robot.subsystems.intakedeploy;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.frc1678.sim.MechanismSim;
import frc.lib.frc1678.sim.PivotSim;
import frc.lib.frc1678.sim.PivotSim.PivotSimConstants;
import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.io.MotorIOTalonFX;
import frc.lib.frc1731.hardware.motor.io.config.CANCoderIOConfigs;
import frc.lib.frc1731.hardware.motor.io.config.TalonFXIOConfigs;
import frc.robot.Ports;

public class IntakeDeployConstants {
    public static final double kGearRatio = 48.0;

    public static final double kStatorCurrentLimit = 60.0;
    public static final double kSupplyCurrentLimit = 40.0;

    public static final Angle kHomeAngle = Rotations.zero();
    public static final Angle kDeployAngle = Rotations.of(-0.13916);
    public static final Angle kEpsilon = Rotations.of(0.01); // TODO - Tune this?

    public static final AngularVelocity kMaxVelocity = RotationsPerSecond.of(2.0);
    public static final AngularAcceleration kMaxAcceleration = RotationsPerSecondPerSecond.of(2.0);

    public static final AngularVelocity kMaxJiggleVelocity = RotationsPerSecond.of(1.0);
    public static final AngularAcceleration kMaxJiggleAcceleration = RotationsPerSecondPerSecond.of(1.0);

    public static final PIDGains kPIDGains = new PIDGains()
        .setP(60)
        .setD(.5)
        .setV(5.76)
        .setA(0.48);

    public static final TalonFXIOConfigs getIOConfigs() {
        return new TalonFXIOConfigs()
            .withPIDGains(kPIDGains)
            .withSensorToMechanismRatio(kGearRatio)
            .withCurrentLimits(kSupplyCurrentLimit, kStatorCurrentLimit)
            .withSoftLimits(kDeployAngle, kHomeAngle)
            .withCANCoder(Ports.kIntakeDeployCANCoderConfig, new CANCoderIOConfigs(0.4541015625, SensorDirectionValue.CounterClockwise_Positive))
            .withMotionMagicSpeeds(kMaxVelocity.in(RotationsPerSecond), kMaxAcceleration.in(RotationsPerSecondPerSecond))
            .brake()
            ;
    }

    public static final MechanismSim getSim() {
        return new PivotSim(
            new PivotSimConstants()

        );
    }

    public static final MotorIOTalonFX getIO() {
        return MotorIOTalonFX.generateKrakenX44FOC(Ports.kIntakeDeployConfig, getIOConfigs());
    }

    @AutoLog
    public static class IntakeDeployIOInputs {
        public double currentPosition, setpointPosition;
        public boolean deployed;
    }
}
