package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import frc.lib.frc1678.sim.PivotSim;
import frc.lib.frc1678.sim.PivotSim.PivotSimConstants;
import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.io.MotorIOTalonFXS;
import frc.lib.frc1731.hardware.motor.io.config.TalonFXSIOConfigs;
import frc.robot.Ports;

public final class HoodConstants {
    public static final double kGearRatio = 1d / (15d / 64d * 32d / 64d * 20d / 380d);
    public static final Angle kHomeAngle = Degrees.zero();
    public static final Angle kMaxAngle = Rotations.of(22);
    public static final Angle kEpsilon = Rotations.one();

    public static final Distance kHoodRadius = Inches.of(8.4); 
    public static final Mass kHoodMass = Pounds.of(5);

    public static final AngularVelocity kMaxVelocity = RotationsPerSecond.of(50);//.div(kGearRatio);
    public static final AngularAcceleration kMaxAcceleration = RotationsPerSecondPerSecond.of(120);//.div(kGearRatio);

    public static final double kHoodMOI = 0.006; // Complete estimate

    public static final double kStatorCurrentLimit = 40.0;
    public static final double kSupplyCurrentLimit = 40.0;

    public static final PIDGains kPIDGains = new PIDGains()
        .setP(4.8)
        .setD(0.1)
        .setV(0.12)
        .setS(0.25)
        .setA(0.01)
    ;

    public static final TalonFXSIOConfigs getIOConfig() {
        return new TalonFXSIOConfigs()
        .withPIDGains(kPIDGains)
        .withCurrentLimits(kSupplyCurrentLimit, kStatorCurrentLimit)
        .withSoftLimits(kHomeAngle, kMaxAngle)
        .withMotionMagicSpeeds(kMaxVelocity.in(RotationsPerSecond), kMaxAcceleration.in(RotationsPerSecondPerSecond))
        // .withSensorToMechanismRatio(kGearRatio)
        .invert()
        .brake();
    }

    public static final PivotSim getSimulation() {
        return new PivotSim(
            new PivotSimConstants()
            .withGearing(kGearRatio)
            .withMotor(DCMotor.getMinion(1))
            .withStartPosition(kHomeAngle)
            .withSoftLimits(kHomeAngle, kMaxAngle)
            .withMOI(kHoodMOI)
        );
    }

    public static final MotorIOTalonFXS getIO() {
        return MotorIOTalonFXS.generateMinion(Ports.kHoodConfig, getIOConfig());
    }

    @AutoLog
    public static class HoodIOInputs {
        public double currentRotations = 0.0;
        public double setpointRotations = 0.0;
        public boolean atSetpoint = false;
    }
}