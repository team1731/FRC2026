package frc.lib.frc1731.hardware.motor.io;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import frc.lib.frc1678.sim.MechanismSim;
import frc.lib.frc1731.hardware.motor.*;
import frc.lib.frc1731.hardware.motor.io.request.*;
import frc.lib.frc1731.hardware.motor.io.request.MotorRequest.RequestType;

/**
 * Hardware abstraction for motors used by team subsystem base classes.
 *
 * <p>Implementations wrap a specific vendor API while exposing common position, velocity, voltage,
 * percent output, soft limit, and simulation operations to robot subsystems.
 */
public abstract class MotorIO {
    private MotorConstants type = null;
    private PortConfig port = null;
    protected MechanismSim mechSim = null;
    private MotorRequest setpointRequest = MotorRequest.withCoastRequest();

    protected MotorIO(MotorConstants type, PortConfig port) {
        this.type = type;
        this.port = port;
    }

    /**
     * Returns the attached mechanism simulation, if one has been configured.
     *
     * @return mechanism simulation model or {@code null}
     */
    public MechanismSim getMechSim() {
        return mechSim;
    }

    /**
     * Returns published constants for the motor model.
     *
     * @return motor constants
     */
    public MotorConstants getConstants() {
        return this.type;
    }

    /**
     * Returns the CAN/port configuration used to construct the motor.
     *
     * @return motor port config
     */
    public PortConfig getPortConfig() {
        return port;
    }

    /**
     * Converts the configured motor type into a WPILib {@link DCMotor} plant model.
     *
     * @param numMotors number of motors represented by the plant
     * @return matching WPILib motor model, or {@code null} if unsupported
     */
    public DCMotor getDCMotor(int numMotors) {
        switch(type) {
            case kKrakenX60:
                return DCMotor.getKrakenX60(numMotors);
            case kKrakenX60FOC:
                return DCMotor.getKrakenX60Foc(numMotors);
            case kKrakenX44:
                return DCMotor.getKrakenX44(numMotors);
            case kKrakenX44FOC:
                return DCMotor.getKrakenX44Foc(numMotors);
            case kFalcon500:
                return DCMotor.getFalcon500(numMotors);
            case kFalcon500FOC:
                return DCMotor.getFalcon500Foc(numMotors);
            case kVortex:
                return DCMotor.getNeoVortex(numMotors);
            case kNeo:
                return DCMotor.getNEO(numMotors);
            case kNeo550:
                return DCMotor.getNeo550(numMotors);
            case kMinion:
                return DCMotor.getMinion(numMotors);
            default: // Type does not exist as a DCMotor so we ignore it
                return null;
        }
    }

    /**
     * Returns the underlying vendor motor object.
     *
     * @param <M> vendor motor type
     * @return wrapped motor controller
     */
    public abstract <M> M getMotor();
    
    /**
     * Attaches a mechanism simulation to this motor IO.
     *
     * @param sim mechanism simulation to drive from motor voltage
     * @param <IO> concrete motor IO type
     * @return this motor IO for chaining
     */
    public abstract <IO extends MotorIO> IO withSimulation(MechanismSim sim);

    /**
     * Resets the motor encoder position.
     *
     * @param position new encoder position in rotations
     */
    public abstract void resetEncoderPosition(Angle position);

    public abstract void updateTrapezoidalSpeeds(AngularVelocity vel, AngularAcceleration accel, int slot);

    public abstract Angle getPosition();

    public abstract AngularVelocity getVelocity();

    public abstract double getAppliedVoltage();

    /** Actual signed duty cycle, in the range -1 to 1. */
    public abstract double getAppliedDutyCycle();

    /**
     * Controller supply current in amps, excluding followers.
     * SPARK implementations estimate this from output current and absolute duty cycle;
     * the estimate excludes controller losses and does not model regeneration.
     */
    public abstract double getSupplyCurrent();

    /** Motor/stator current in amps for this controller, excluding followers. */
    public abstract double getStatorCurrent();

    /**
     * Returns configured forward software limit.
     *
     * @return forward limit in rotations or vendor-native position units
     */
    public abstract Angle getForwardLimit();

    /**
     * Returns configured reverse software limit.
     *
     * @return reverse limit in rotations or vendor-native position units
     */
    public abstract Angle getReverseLimit();

    public abstract void coast();

    public abstract void brake();

    public abstract void setPercent(double setpoint);

    public abstract void setVoltage(double setpoint);

    public abstract void setPosition(Angle setpoint, int slot);

    public abstract void setPositionTrapezoidal(Angle setpoint, int slot);

    public abstract void setVelocity(AngularVelocity setpoint, int slot);

    /**
     * Advances any attached simulation and feeds simulated state back to the vendor API.
     */
    public abstract void simPeriodic();

    public AngularVelocity getMaxFreeSpeed() {
        return getConstants().kMaxVelocity;
    }

    /** Returns the type last submitted through applyRequest; direct hardware calls are not tracked. */
    public RequestType getRequestType() {
        if (setpointRequest == null) return RequestType.kIdle;
        return this.setpointRequest.type;
    }

    public double getRequestSetpointAsDouble() {
        if (setpointRequest == null) return 0.0;
        return this.setpointRequest.getBaseValue();
    }

    public MotorRequest getRequest() {
        if (setpointRequest == null) return new BrakeRequest();
        return this.setpointRequest;
    }

    /** Applies a request, then records it for telemetry. Hardware methods receive slots explicitly. */
    public void applyRequest(MotorRequest request) {
        java.util.Objects.requireNonNull(request, "request").apply(this);
        this.setpointRequest = request;
    }
}
