package frc.robot.subsystems.shooter.flywheel;

import frc.lib.frc1731.hardware.motor.ctre.MotorIOTalonFX;
import frc.lib.frc1731.math.LoggedTunableNumber;
import frc.lib.frc1731.subsystem.VelocitySubsystem;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.*;

import edu.wpi.first.wpilibj2.command.Command;

public class FlywheelSubsystem extends VelocitySubsystem<MotorIOTalonFX> {
    private LoggedTunableNumber flywheelTunedSetpoint = new LoggedTunableNumber("Flywheel Setpoint RPS", 50, () -> true);

    public FlywheelSubsystem(boolean enabled) {
        super(enabled);
        super.withTolerance(kEpsilon);
        super.withSysId(1d, 4d, 10d);
        super.withSimulation(kSimConstants, kVelocityGains);
    }

    @Override
    protected void initializeHardware() {
        motor = new MotorIOTalonFX(kLeftFlywheelConfig);
        motor.withPIDGains(kVelocityGains);
        motor.withStatorCurrentLimit(kCurrentLimit);
    }

    @Override
    public void periodicTelemetry() {
        logger.log("Current Velocity RPS", getVelocity().in(RotationsPerSecond));
        logger.log("Target Velocity RPS", getTargetVelocity().in(RotationsPerSecond));
        logger.log("At Target Velocity", atTargetVelocity());
    }

    public Command tuneShotCommand() {
        return super.setVelocityCommand(() -> RotationsPerSecond.of(flywheelTunedSetpoint.get()));
    }
}