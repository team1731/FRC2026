package frc.lib.frc1731.subsystem;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.hardware.motor.io.MotorIO;
import frc.lib.frc1731.hardware.motor.io.request.*;

/**
 * Base for velocity-controlled rollers, feeders, and flywheels. Use {@link #applyRequest}
 * for custom slots and tolerances. Subclasses implement {@link #updateInputs()}.
 *
 * @param <IO> motor IO implementation used by the mechanism
 */
public abstract class BaseVelocitySubsystem<IO extends MotorIO> extends BaseMotorSubsystem<IO> {
    public BaseVelocitySubsystem(IO motor) {
        super(motor);
        super.setDefaultCommand(stop());
    }

    public AngularVelocity getVelocity() {
        return getMotor().getVelocity();
    }

    public AngularVelocity getSetpoint() {
        return getMotor().getRequestType().isVelocityRequest() ? RadiansPerSecond.of(getMotor().getRequestSetpointAsDouble()) : getVelocity();
    }

    /** Continuously commands a fixed angular velocity until interrupted. */
    public Command setVelocity(AngularVelocity setpoint) {
        return setVelocity(() -> setpoint);
    }

    /** Samples the target each scheduler cycle. Defaults to slot zero and exact tolerance. */
    public Command setVelocity(Supplier<AngularVelocity> setpoint) {
        return applyRequest(() -> new VelocityRequest().withVelocity(setpoint.get())).withName("SetVelocity");
    }

    /** Continuously commands a fixed angular velocity until interrupted. */
    public Command setVelocityWithEpsilon(AngularVelocity setpoint, AngularVelocity epsilon) {
        return setVelocityWithEpsilon(() -> setpoint, epsilon);
    }

    /** Samples the target each scheduler cycle. Defaults to slot zero and exact tolerance. */
    public Command setVelocityWithEpsilon(Supplier<AngularVelocity> setpoint, AngularVelocity epsilon) {
        return applyRequest(() -> new VelocityRequest().withVelocity(setpoint.get()).withEpsilonThreshold(epsilon)).withName("SetVelocity");
    }

    /** Continuously commands open-loop voltage in volts. */
    public Command setVoltage(double voltage) {
        return setVoltage(() -> voltage);
    }

    /** Samples open-loop voltage each scheduler cycle, clamped to [-12, 12] volts. */
    public Command setVoltage(DoubleSupplier voltage) {
        return applyRequest(() -> new VoltageRequest(Math.max(-12, Math.min(12, voltage.getAsDouble()))))
                .withName("SetVoltage");
    }

    /** Continuously commands a duty cycle. */
    public Command setPercent(double percent) {
        return setPercent(() -> percent);
    }

    /** Samples the duty cycle each scheduler cycle, clamped to [-1, 1]. */
    public Command setPercent(DoubleSupplier percent) {
        return applyRequest(() -> new DutyCycleRequest(Math.max(-1, Math.min(1, percent.getAsDouble()))))
                .withName("SetPercent");
    }

    /** Continuously commands zero output until interrupted. */
    public Command stop() {
        return setPercent(0).withName("Stop");
    }
}