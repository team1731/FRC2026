package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.hardware.motor.io.MotorIOTalonFX;
import frc.lib.frc1731.subsystem.BaseVelocitySubsystem;

public class FlywheelSubsystem extends BaseVelocitySubsystem<MotorIOTalonFX> {
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    public FlywheelSubsystem() {
        super(FlywheelConstants.getIO());
        super.setDefaultCommand(stop());
    }

    @Override
    public void periodicTelemetry() {
        inputs.currentVelocity = getVelocity().in(RotationsPerSecond);
        inputs.setpointVelocity = getSetpoint().in(RotationsPerSecond);
        inputs.atSetpoint = atSetpoint();
        inputs.isWarmingUp = !getSetpoint().equals(RotationsPerSecond.zero());
        logger.processInputs(inputs);
    }

    public Command shoot(Supplier<AngularVelocity> setpoint) {
        return setVelocityWithEpsilon(setpoint, FlywheelConstants.kEpsilon);
    }

    public Command shoot(AngularVelocity setpoint) {
        return setVelocityWithEpsilon(setpoint, FlywheelConstants.kEpsilon);
    }

    public Command warmup() {
        return this.shoot(FlywheelConstants.kWarmupVelocity);
    }
}