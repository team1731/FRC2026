package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.frc1678.sim.PivotSim.PivotSimConstants;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.lib.frc1731.sim.SimpleAngularMotorSim;
import frc.robot.Robot;
import frc.robot.subsystems.BaseSubsystem;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.hood.HoodConstants.*;

public class HoodSubsystem extends BaseSubsystem {
    private MotorIOTalonFX motor;
    private Angle targetAngle = Degrees.zero();
    // private ArmFeedforward simFF = new ArmFeedforward(kPositionGains.kS, kPositionGains.kG, kPositionGains.kV, kPositionGains.kA);

    // private SingleJointedArmSim sim = new SingleJointedArmSim(
    //     kDCMotor,
    //     1d/kGearRatio,
    //     0.5 * kHoodMass.in(Kilograms) * Math.pow(kHoodRadius.in(Meters), 2), // 0.0004
    //     kHoodRadius.in(Meters), 
    //     kStartAngle.in(Radians), 
    //     kMaxAngle.in(Radians), 
    //     false, 
    //     kStartAngle.in(Radians)
    // );

    private SimpleAngularMotorSim sim = new SimpleAngularMotorSim(
        new PivotSimConstants()
            .withMotor(DCMotor.getMinion(1))
            .withPhysics(1d/kGearRatio, 0.02, false)
            .withConstraints(kStartAngle.in(Degrees), kMaxAngle.in(Degrees), kStartAngle.in(Degrees), kHoodRadius.in(Meters)), 
        kPositionGains
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
    //             log.motor("hood")
    //             .voltage(Robot.isSimulation() ? sim.getAppliedVoltage() : Volts.of(motor.getAppliedVoltage()))
    //             .angularPosition(Robot.isSimulation() ? sim.getMechanismAngle() : Rotations.of(motor.getRotations()))
    //             .angularVelocity(Robot.isSimulation() ? sim.getMechanismVelocity() : RotationsPerSecond.of(motor.getVelocityRPS()));
    //         },
    //         this
    //     )
    // );

    public HoodSubsystem(boolean enabled) {
        super(enabled);
        if (!isEnabled()) return;
        motor = new MotorIOTalonFX(kLeftHoodConfig);
        motor.withPIDGains(kPositionGains);
        motor.resetEncoderPosition(kStartRotations.in(Rotations));
        motor.setSoftLimits(kStartAngle.in(Rotations), kMaxAngle.in(Rotations));
    }

    public Angle getMotorAngle() {
        // if (Robot.isSimulation()) return kConverter.toMotor(Radians.of(sim.getAngleRads()));
        if (Robot.isSimulation()) return kConverter.toMotor(sim.getMechanismAngle());
        return Rotations.of(motor.getRotations());
    }

    public Angle getHoodAngle() {
        if (Robot.isSimulation()) return sim.getMechanismAngle();
        return kConverter.toMechanism(Rotations.of(motor.getRotations()));
    }

    public Angle getTargetAngle() {
        return targetAngle;
    }

    @SuppressWarnings("unused")
    private void setVoltage(Voltage volts) {
        motor.setVoltage(volts.in(Volts));
        if (Robot.isSimulation()) sim.setVoltage(volts);
    }

    public boolean atTargetAngle() {
        return getHoodAngle().isNear(targetAngle, kEpsilon);
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Position", getMotorAngle().in(Rotations));
        logger.log("Current Degrees", getHoodAngle().in(Degrees));
        logger.log("Target Position", kConverter.toMotor(getTargetAngle()).in(Rotations));
        logger.log("Target Degrees", getTargetAngle().in(Degrees));
        logger.log("At Target Angle", atTargetAngle());
        sim.periodic();
    }

    public Command setAngleCommand(Angle targetAngle) {
        return run(() -> {
            this.targetAngle = targetAngle;
            motor.setPosition(kConverter.toMotor(targetAngle).in(Rotations));
            sim.setMechanismAngle(targetAngle);
        });
    }

    // public Command setHoodAngleCommand(Angle targetAngle) {
    //     return run(() -> {
    //         this.targetAngle = targetAngle;
    //         motor.setPosition(kConverter.toMotor(targetAngle).in(Rotations));
    //         if (Robot.isReal()) return;
    //         double outputCurrent = kPositionGains.toPIDController().calculate(getMotorAngle().in(Rotations), kConverter.toMotor(getTargetAngle()).in(Rotations))
    //                                     + simFF.calculate(kConverter.toMotor(getTargetAngle()).in(Rotations), 0);

    //         outputCurrent = Utils.clamp(outputCurrent, kCurrentLimit);

    //         double appliedVolts = kDCMotor.getVoltage(outputCurrent, sim.getVelocityRadPerSec());
    //         appliedVolts = Utils.clamp(appliedVolts, 12d);
    //         sim.setInputVoltage(appliedVolts);
    //     });
    // }

    public Command setManualCommand(double percent) {
        return run(() -> {
            motor.setPercentOutput(percent);
            sim.setVoltage(Volts.of(percent*12d));
        });
    }

    public Command stowHoodCommand() {
        return setAngleCommand(kStartAngle);
    }
}