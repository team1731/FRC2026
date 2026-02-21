package frc.robot.subsystems.shooter.turret;

import frc.lib.frc1731.Utils;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.lib.frc6328.FieldConstants;
import frc.lib.frc6328.FieldConstants.Hub;
import frc.robot.Robot;
import frc.robot.subsystems.BaseSubsystem;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;

public class TurretSubsystem extends BaseSubsystem {
    private MotorIOTalonFX motor;
    private Angle targetAngle = Degrees.zero();
    private ArmFeedforward simFF = new ArmFeedforward(kPositionGains.kS, kPositionGains.kG, kPositionGains.kV, kPositionGains.kA);

    private SingleJointedArmSim sim = new SingleJointedArmSim(
        kDCMotor,
        1d/kGearRatio,
        0.5 * kTurretMass.in(Kilograms) * Math.pow(kTurretRadius.in(Meters), 2), // 0.0004
        kTurretRadius.in(Meters), 
        kMinAngle.in(Radians), 
        kMaxAngle.in(Radians),
        false, 
        0d
    );

    // private SysIdRoutine routine = new SysIdRoutine(
    //     new SysIdRoutine.Config(
    //         Volts.of(0.25d).per(Second), 
    //         Volts.of(0.25d),
    //         null
    //     ), 
    //     new SysIdRoutine.Mechanism(
    //         volts -> setVoltage(volts), 
    //         log -> {
    //             log.motor("turret")
    //             .voltage(Robot.isSimulation() ? sim.getAppliedVoltage() : Volts.of(motor.getAppliedVoltage()))
    //             .angularPosition(Robot.isSimulation() ? sim.getMechanismAngle() : Rotations.of(motor.getRotations()))
    //             .angularVelocity(Robot.isSimulation() ? sim.getMechanismVelocity() : RotationsPerSecond.of(motor.getVelocityRPS()));
    //         },
    //         this
    //     )
    // );

    public TurretSubsystem(boolean enabled) {
        super(enabled);
        if (!enabled) return;
        motor = new MotorIOTalonFX(kLeftPortConfigs);
        motor.withPIDGains(kPositionGains);
        motor.setSoftLimits(kMinAngle.in(Rotations), kMaxAngle.in(Rotations));
    }

    public Angle getTargetAngle() {
        return targetAngle;
    }

    public Angle getTurretAngle() {
        if (Robot.isSimulation()) return Radians.of(sim.getAngleRads());
        return kConverter.toMechanism(Rotations.of(motor.getRotations()));
    }

    public Angle getMotorAngle() {
        if (Robot.isSimulation()) return kConverter.toMotor(Radians.of(sim.getAngleRads()));
        return Rotations.of(motor.getRotations());
    }

    public boolean atTargetAngle() {
        return getTurretAngle().isNear(getTargetAngle(), kEpsilon);
    }

    @SuppressWarnings("unused")
    private void setVoltage(Voltage volts) {
        motor.setVoltage(volts.in(Volts));
        if (Robot.isSimulation()) sim.setInputVoltage(volts.in(Volts));
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Rotations", motor.getRotations());
        logger.log("Current Degrees", getTurretAngle().in(Degrees));
        logger.log("Target Rotations", kConverter.toMotor(getTargetAngle()).in(Rotations));
        logger.log("Target Degrees", getTargetAngle().in(Degrees));
        logger.log("At Target Angle", atTargetAngle());
        sim.update(Robot.CLOCK.dt());
    }

    public Command setTurretAngleCommand(Angle targetAngle) {
        return run(() -> {
            this.targetAngle = targetAngle;
            motor.setPosition(kConverter.toMotor(targetAngle).in(Rotations));
            if (Robot.isReal()) return;
            double outputCurrent = kPositionGains.toPIDController().calculate(getMotorAngle().in(Rotations), kConverter.toMotor(getTargetAngle()).in(Rotations))
                + simFF.calculate(kConverter.toMotor(getTargetAngle()).in(Radians), kConverter.toMotor(Rotations.of(sim.getVelocityRadPerSec()/(2*Math.PI))).in(Rotations));
            outputCurrent = Utils.clamp(outputCurrent, kCurrentLimit);

            double appliedVolts = kDCMotor.getVoltage(outputCurrent, sim.getVelocityRadPerSec());
            appliedVolts = Utils.clamp(appliedVolts, 12d);
            sim.setInputVoltage(appliedVolts);
        });
    }

    public Command setManualCommand(double percent) {
        return run(() -> {
            motor.setPercentOutput(percent);
            sim.setInputVoltage(percent*12d);
        });
    }

    public Command aimAtHub(Supplier<Pose2d> swervePose) {
        return run(() -> {
            Translation2d targetPose = Utils.flip(FieldConstants.Hub.topCenterPoint.toTranslation2d());
            Pose2d robotPose = swervePose.get();

            
        });
    }
}