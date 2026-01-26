package frc.robot.subsystems.hood;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.frc1731.hardware.MotorIOTalonFX;
import frc.lib.frc1731.hardware.motor.PortConfig;
import frc.lib.frc1731.sim.SimpleAngularMotorSim;
import frc.lib.frc1731.subsystem.BaseSubsystem;
import frc.lib.frc1731.subsystem.converter.AngularSubsystemConverter;
import frc.robot.Robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.hood.HoodConstants.*;

public class HoodSubsystem extends BaseSubsystem {
    private MotorIOTalonFX motor;
    private double targetDegrees = 0.0;
    private SimpleAngularMotorSim sim = new SimpleAngularMotorSim(kSimConstants, kPositionGains);
    private AngularSubsystemConverter converter = new AngularSubsystemConverter(kGearRatio);

    private SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(0.25d).per(Second), 
            Volts.of(0.25d),
            null
        ), 
        new SysIdRoutine.Mechanism(
            volts -> setVoltage(volts), 
            log -> {
                log.motor("hood")
                .voltage(Robot.isSimulation() ? sim.getAppliedVoltage() : Volts.of(motor.getAppliedVoltage()))
                .angularPosition(Robot.isSimulation() ? sim.getMechanismAngle() : Rotations.of(motor.getRotations()))
                .angularVelocity(Robot.isSimulation() ? sim.getMechanismVelocity() : RotationsPerSecond.of(motor.getVelocityRPS()));
            },
            this
        )
    );

    public HoodSubsystem(boolean enabled) {
        super(enabled);
        if (!isEnabled()) return;
        motor = new MotorIOTalonFX(new PortConfig(0));
        motor.withPIDGains(kPositionGains);
        motor.resetEncoderPosition(converter.toMotor(Degrees.of(kStartDegrees)).in(Degrees));
    }

    public Angle getMotorAngle() {
        if (Robot.isSimulation()) return converter.toMotor(sim.getMechanismAngle());
        return Rotations.of(motor.getRotations());
    }

    public double getDegrees() {
        if (Robot.isSimulation()) return sim.getMechanismAngle().in(Degrees);
        return converter.toMechanism(Rotations.of(motor.getRotations())).in(Degrees);
    }

    private void setVoltage(Voltage volts) {
        motor.setVoltage(volts.in(Volts));
        if (Robot.isSimulation()) sim.setVoltage(volts);
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Position", getMotorAngle().in(Rotations));
        logger.log("Current Degrees", getDegrees());
        logger.log("Target Position", converter.toMotor(Degrees.of(targetDegrees)).in(Rotations));
        logger.log("Target Degrees", targetDegrees);
        sim.periodic();
    }

    public Command setDegreesCommand(double degrees) {
        return run(() -> {
            targetDegrees = degrees;
            motor.setPosition(converter.toMotor(Degrees.of(targetDegrees)).in(Rotations));
            if (Robot.isSimulation()) sim.setMechanismAngle(Degrees.of(degrees));
        });
    }

    public Command sysIdDynamicCommand(boolean forward) {
        return Commands.either(
            routine.dynamic(Direction.kForward), 
            routine.dynamic(Direction.kReverse), 
            () -> forward
        );
    }

    public Command sysIdQuasistaticCommand(boolean forward) {
        return Commands.either(
            routine.quasistatic(Direction.kForward), 
            routine.quasistatic(Direction.kReverse), 
            () -> forward
        );
    }
}