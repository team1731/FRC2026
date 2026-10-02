package frc.lib.frc1731.subsystem;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1678.util.Util.DistanceAngleConverter;
import frc.lib.frc1731.hardware.motor.io.MotorIO;
import frc.lib.frc1731.hardware.motor.io.request.PositionRequest;

/**
 * Base for position-controlled mechanisms. Use {@link #applyRequest} for custom slots,
 * tolerances, linear targets, or motion profiles. Subclasses implement {@link #updateInputs()}.
 *
 * @param <IO> motor IO implementation used by the mechanism
 */
public abstract class BaseServoSubsystem<IO extends MotorIO, I extends LoggableInputs> extends BaseMotorSubsystem<IO, I> {
    private DistanceAngleConverter converter = null;

    protected BaseServoSubsystem(IO motor, I inputs) {
        super(motor, inputs);
    }

    protected BaseServoSubsystem(IO motor, DistanceAngleConverter converter, I inputs) {
        super(motor, inputs);
        this.converter = converter;
    }

    public Angle getPosition() {
        return getMotor().getPosition();
    }

    public Distance getLinearPosition() {
        if (converter == null) return Inches.zero();
        return converter.toDistance(getPosition());
    }

    public Angle getSetpoint() {
        return getMotor().getRequestType().isPositionRequest() ? Radians.of(getMotor().getRequestSetpointAsDouble()) : getPosition();
    }

    public Distance getLinearSetpoint() {
        return getMotor().getRequestType().isPositionRequest() ? Meters.of(getMotor().getRequestSetpointAsDouble()) : getLinearPosition();
    }

    public boolean isNear(Angle setpoint, Angle epsilon) {
        return getPosition().isNear(setpoint, epsilon);
    }

    public boolean isNearHome(Angle epsilon) {
        return isNear(Degrees.zero(), epsilon);
    }

    public boolean isNear(Distance setpoint, Distance epsilon) {
        return getLinearPosition().isNear(setpoint, epsilon);
    }

    public boolean isNearHome(Distance epsilon) {
        return getLinearPosition().isNear(Inches.zero(), epsilon);
    }

    /** Resets encoder feedback without moving the mechanism; ignored while inactive. */
    public void resetPosition(Angle position) {
        if (isActiveSubsystem()) {
            getMotor().resetEncoderPosition(position);
        }
    }

    /** Continuously commands a fixed angular position until interrupted. */
    public Command setPosition(Angle position) {
        return setPosition(() -> position);
    }

    /** Samples the target each scheduler cycle. Defaults to slot zero and exact tolerance. */
    public Command setPosition(Supplier<Angle> position) {
        return applyRequest(() -> new PositionRequest().withPosition(position.get())).withName("SetPosition");
    }

    /** Captures the position when scheduled and holds it until interrupted. */
    public Command stop() {
        PositionRequest request = new PositionRequest();
        return runOnce(() -> new PositionRequest().withPosition(getPosition()))
                .andThen(applyRequest(() -> request)).withName("HoldPosition");
    }

    @AutoLog
    public static class BaseAngularServoInputs {
        public Angle currentPosition, setpointPosition;
        public boolean atSetpoint;
    }

    @AutoLog
    public static class BaseLinearServoInputs {
        public Distance currentPosition, setpointPosition;
        public boolean atSetpoint;
    }
}
