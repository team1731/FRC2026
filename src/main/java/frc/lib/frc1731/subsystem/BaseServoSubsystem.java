package frc.lib.frc1731.subsystem;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.hardware.motor.io.MotorIO;
import frc.lib.frc1731.hardware.motor.io.request.*;

/**
 * Base for position-controlled mechanisms. Use {@link #applyRequest} for custom slots,
 * tolerances, linear targets, or motion profiles. Subclasses implement {@link #updateInputs()}.
 *
 * @param <IO> motor IO implementation used by the mechanism
 */
public abstract class BaseServoSubsystem<IO extends MotorIO> extends BaseMotorSubsystem<IO> {
    protected BaseServoSubsystem(IO motor) {
        super(motor);
    }

    public Angle getPosition() {
        return getMotor().getPosition();
    }

    public Angle getSetpoint() {
        return getMotor().getRequestType().isPositionRequest() ? Radians.of(getMotor().getRequestSetpointAsDouble()) : getPosition();
    }

    public boolean isNear(Angle setpoint, Angle epsilon) {
        return getPosition().isNear(setpoint, epsilon);
    }

    public boolean isNearHome(Angle epsilon) {
        return isNear(Degrees.zero(), epsilon);
    }

    /** Resets encoder feedback without moving the mechanism; ignored while inactive. */
    public void resetPosition(Angle setpoint) {
        if (isActiveSubsystem()) {
            getMotor().resetEncoderPosition(setpoint);
        }
    }

    /** Continuously commands a fixed angular position until interrupted. */
    public Command setPosition(Angle setpoint) {
        return setPosition(() -> setpoint);
    }

    /** Samples the target each scheduler cycle. Defaults to slot zero and exact tolerance. */
    public Command setPosition(Supplier<Angle> setpoint) {
        return applyRequest(() -> new TrapezoidalPositionRequest().withPosition(setpoint.get())).withName("SetPosition");
    }

    public Command setPositionWithEpsilon(Supplier<Angle> setpoint, Angle epsilon) {
        return applyRequest(() -> new TrapezoidalPositionRequest().withPosition(setpoint.get()).withEpsilonThreshold(epsilon)).withName("SetPosition");
    }

    public Command setPositionWithEpsilon(Angle setpoint, Angle epsilon) {
        return this.setPositionWithEpsilon(() -> setpoint, epsilon);
    }

    /** Continuously commands a fixed angular position until interrupted. */
    public Command setPositionWithSpeeds(Supplier<Angle> setpoint, AngularVelocity vel, AngularAcceleration accel) {
        return applyRequest(() -> new TrapezoidalPositionRequest().withPosition(setpoint.get()).withSpeeds(vel, accel)).withName("SetPosition");
    }

    /** Continuously commands a fixed angular position until interrupted. */
    public Command setPositionWithSpeeds(Angle setpoint, AngularVelocity vel, AngularAcceleration accel) {
        return setPositionWithSpeeds(() -> setpoint, vel, accel);
    }

    public Command setPositionWithSpeedsAndEpsilon(Supplier<Angle> setpoint, AngularVelocity vel, AngularAcceleration accel, Angle epsilon) {
        return applyRequest(() -> new TrapezoidalPositionRequest().withPosition(setpoint.get()).withSpeeds(vel, accel).withEpsilonThreshold(epsilon)).withName("SetPosition");
    }

    public Command setPositionWithSpeedsAndEpsilon(Angle setpoint, AngularVelocity vel, AngularAcceleration accel, Angle epsilon) {
        return this.setPositionWithSpeedsAndEpsilon(() -> setpoint, vel, accel, epsilon);
    }

    /** Captures the position when scheduled and holds it until interrupted. */
    public Command stop() {
        TrapezoidalPositionRequest request = new TrapezoidalPositionRequest();
        return runOnce(() -> request.withPosition(getPosition()))
            .andThen(applyRequest(() -> request)).withName("HoldPosition");
    }
}
