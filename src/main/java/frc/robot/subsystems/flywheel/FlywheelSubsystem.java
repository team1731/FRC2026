package frc.robot.subsystems.flywheel;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.frc1731.Utils;
import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.lib.frc1731.math.LoggedTunableNumber;
import frc.lib.frc1731.sim.SimpleVelocitySim;
import frc.robot.Robot;
import frc.robot.subsystems.BaseSubsystem;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.flywheel.FlywheelConstants.*;

public class FlywheelSubsystem extends BaseSubsystem {
    private MotorIOTalonFX motor;
    private SimpleVelocitySim sim = SimpleVelocitySim.buildKrakenX60Sim(1, kGearRatio, kFlywheelRadius, kFlywheelMass, kVelocityGains);
    private double setpointVelocityRPS = 0.0;
    private LoggedTunableNumber flywheelTunedSetpoint = new LoggedTunableNumber("Flywheel Setpoint RPS", 20d, () -> true);

    private SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, 
            Volts.of(4d), 
            null
        ), 
        new SysIdRoutine.Mechanism(
            volts -> setVoltage(volts), 
            log -> {
                // Record a frame for the flywheel motor.
                log.motor("flywheel")
                .voltage(Robot.isSimulation() ? sim.getAppliedVoltage() : Volts.of(motor.getAppliedVoltage()))
                .angularPosition(Robot.isSimulation() ? sim.getPosition() : Rotations.of(motor.getRotations()))
                .angularVelocity(Robot.isSimulation() ? sim.getVelocity() : RotationsPerSecond.of(motor.getVelocityRPS()));
            },
            this
        )
    );

    public FlywheelSubsystem(boolean enabled) {
        super(enabled);
        if (!enabled) return;
        motor = new MotorIOTalonFX(kLeftFlywheelConfig);
        motor.withPIDGains(kVelocityGains);
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Velocity RPS", getVelocityRPS());
        logger.log("Target Velocity RPS", getTargetVelocityRPS());
        logger.log("At Target Velocity", atTargetVelocity());
        sim.periodic();
    }

    public double getVelocityRPS() {
        if (Robot.isSimulation()) return sim.getVelocity().in(RotationsPerSecond);
        return motor.getVelocityRPS();
    }

    public double getTargetVelocityRPS() {
        return setpointVelocityRPS;
    }

    public boolean atTargetVelocity() {
        return Utils.isWithin(getVelocityRPS(), getTargetVelocityRPS(), kEpsilon);
    }

    private void setVoltage(Voltage volts) {
        motor.setVoltage(volts.in(Volts));
        if (Robot.isSimulation()) sim.setVoltage(volts); 
    }

    public Command setVelocityCommand(double velocityRPS) {
        return run(() -> {
            this.setpointVelocityRPS = velocityRPS;
            motor.setVelocityRPS(velocityRPS);
            sim.setVelocity(RotationsPerSecond.of(velocityRPS));
        }).withName("SetVelocity");
    }

    public Command tuneableShotCommand() {
        return run(() -> {
            this.setpointVelocityRPS = flywheelTunedSetpoint.get();
            motor.setVelocityRPS(setpointVelocityRPS);
            sim.setVelocity(RotationsPerSecond.of(setpointVelocityRPS));
        }).withName("Shoot");
    }

    public Command stallCommand() {
        return this.setVelocityCommand(kStallVelocityRPS)
        .withName("Stall");
    }

    public Command stopCommand() {
        return setVelocityCommand(0d)
        .withName("Stop");
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