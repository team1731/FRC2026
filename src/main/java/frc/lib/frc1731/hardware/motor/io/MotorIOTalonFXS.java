package frc.lib.frc1731.hardware.motor.io;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.sim.TalonFXSSimState;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.frc1678.sim.MechanismSim;
import frc.lib.frc1731.hardware.motor.*;
import frc.robot.Robot;
import frc.lib.frc1731.hardware.motor.io.config.TalonFXSIOConfigs;
import frc.lib.frc1731.hardware.motor.io.config.MotorGroupConfig;

/**
 * {@link MotorIO} implementation for CTRE TalonFXS-based motors.
 *
 * <p>Use the static factory method matching the physical motor type so base subsystem helpers can
 * use the correct motor constants for setpoints and simulation.
 */
public class MotorIOTalonFXS extends MotorIO implements AutoCloseable {
    private final java.util.List<TalonFXS> followers = new java.util.ArrayList<>();
    private TalonFXS motor;
    private TalonFXSConfiguration cfg;
    private TalonFXSConfigurator configurator;
    private DynamicMotionMagicVoltage magicOutput;

    private MotorIOTalonFXS(MotorConstants constants, PortConfig port, TalonFXSConfiguration config) {
        super(constants, port);
        this.motor = new TalonFXS(port.kPort, port.kBus);
        this.cfg = config;
        this.cfg.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        this.configurator = motor.getConfigurator();
        this.configurator.apply(cfg);

        this.magicOutput = new DynamicMotionMagicVoltage(0, 0, 0);
        this.magicOutput.Velocity = cfg.MotionMagic.MotionMagicCruiseVelocity;
        this.magicOutput.Acceleration = cfg.MotionMagic.MotionMagicAcceleration;
    }

    /**
     * Creates a Kraken X60 TalonFXS wrapper with a custom CTRE configuration.
     *
     * @param port CAN bus and ID for the motor
     * @param config TalonFXS configuration to apply on construction
     * @return configured TalonFXS motor IO
     */
    public static MotorIOTalonFXS generateMinion(PortConfig port, TalonFXSConfiguration config) {
        return new MotorIOTalonFXS(MotorConstants.kKrakenX60, port, config);
    }

    /** Creates a leader and configured followers; pass the builder without calling build(). */
    public static MotorIOTalonFXS generateMinion(PortConfig port, TalonFXSIOConfigs configs) {
        var group = configs.buildMotorGroup(port.kPort);
        var io = new MotorIOTalonFXS(MotorConstants.kKrakenX60, port, group.leader());
        try {
            io.configureFollowers(group);
            return io;
        } catch (RuntimeException failure) {
            io.close();
            throw failure;
        }
    }

    /**
     * Creates a Kraken X44 FOC TalonFXS wrapper with a custom CTRE configuration.
     */
    public static MotorIOTalonFXS generateMinionFOC(PortConfig port, TalonFXSConfiguration config) {
        return new MotorIOTalonFXS(MotorConstants.kKrakenX44FOC, port, config);
    }

    /** Creates a leader and configured followers; pass the builder without calling build(). */
    public static MotorIOTalonFXS generateMinionFOC(PortConfig port, TalonFXSIOConfigs configs) {
        var group = configs.buildMotorGroup(port.kPort);
        var io = new MotorIOTalonFXS(MotorConstants.kKrakenX44FOC, port, group.leader());
        try {
            io.configureFollowers(group);
            return io;
        } catch (RuntimeException failure) {
            io.close();
            throw failure;
        }
    }

    @Override
    @SuppressWarnings("unchecked")
    public TalonFXS getMotor() {
        return this.motor;
    }

    @Override
    @SuppressWarnings("unchecked")
    public MotorIOTalonFXS withSimulation(MechanismSim sim) {
        this.mechSim = sim;
        return this;
    }

    @Override
    public void resetEncoderPosition(Angle position) {
        this.motor.setPosition(position);
    }

    @Override
    public void updateTrapezoidalSpeeds(AngularVelocity vel, AngularAcceleration accel, int slot) {
        this.magicOutput.Velocity = vel.in(RotationsPerSecond);
        this.magicOutput.Acceleration = accel.in(RotationsPerSecondPerSecond);

        // DynamicMotionMagicVoltage carries these constraints in the control request.
    }

    @Override
    public Angle getPosition() {
        return this.motor.getPosition().getValue();
    }

    @Override
    public AngularVelocity getVelocity() {
        return this.motor.getVelocity().getValue();
    }

    @Override
    public double getAppliedDutyCycle() {
        return motor.getDutyCycle().getValueAsDouble();
    }

    @Override
    public double getSupplyCurrent() {
        return motor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public double getStatorCurrent() {
        return motor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public double getAppliedVoltage() {
        return this.motor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public Angle getForwardLimit() {
        return Rotations.of(this.cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold);
    }

    @Override
    public Angle getReverseLimit() {
        return Rotations.of(this.cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold);
    }

    @Override
    public void coast() {
        motor.setControl(new CoastOut());
    }

    @Override
    public void brake() {
        motor.setControl(new StaticBrake());
    }

    @Override
    public void setPercent(double percent) {
        this.motor.set(percent);
    }

    @Override
    public void setVoltage(double volts) {
        this.motor.setControl(new VoltageOut(volts));
    }

    @Override
    public void setPosition(Angle setpoint, int slot) {
        this.motor.setControl(new PositionVoltage(setpoint).withSlot(slot));
    }

    @Override
    public void setPositionTrapezoidal(Angle setpoint, int slot) {
        this.motor.setControl(magicOutput.withPosition(setpoint).withSlot(slot));
    }

    @Override
    public void setVelocity(AngularVelocity setpoint, int slot) {
        this.motor.setControl(new VelocityVoltage(setpoint).withSlot(slot));
    }

    @Override
    public void simPeriodic() {
        if (!Robot.isSimulation() || mechSim == null) return; // Only move forward if we are in simulation mode and mechanism sim is initialized
        TalonFXSSimState simState = this.motor.getSimState();

        // Battery sag limits how much voltage the motor controller can actually put out
        simState.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Whatever voltage the TalonFXS's control loop decided to apply (based on the rotor
        // state we fed back last loop), hand it to the physics sim as the plant input
        mechSim.setVoltage(Volts.of(motor.getSimState().getMotorVoltage()));
        mechSim.simulate();

        // Feed the resulting mechanism-referenced position/velocity back as rotor units
        // (mechanismToRotor() applies the same gearing the sim was constructed with) so the
        // TalonFXS's internal control loop sees a physically accurate plant next loop
        simState.setRawRotorPosition(mechSim.mechanismToRotor(mechSim.getPosition()).in(Rotations));
        simState.setRotorVelocity(mechSim.mechanismToRotor(mechSim.getVelocity()).in(RotationsPerSecond));
    }

    private void configureFollowers(MotorGroupConfig<TalonFXSConfiguration> group) {
        for (var spec : group.followers()) {
            var follower = new TalonFXS(spec.deviceId(), getPortConfig().kBus);
            followers.add(follower);
            var status = follower.getConfigurator().apply(spec.config());
            if (!status.isOK()) throw new IllegalStateException("Follower configuration failed: " + status);
            status = follower.setControl(new Follower(motor.getDeviceID(), spec.inverted()
                ? com.ctre.phoenix6.signals.MotorAlignmentValue.Opposed
                : com.ctre.phoenix6.signals.MotorAlignmentValue.Aligned));
            if (!status.isOK()) throw new IllegalStateException("Follower request failed: " + status);
        }
    }

    /** Retained controllers for diagnostics. Direct control can break hardware following. */
    public java.util.List<TalonFXS> getFollowerMotors() {
        return java.util.List.copyOf(followers);
    }

    /** Releases the leader and followers when this IO is no longer used. */
    @Override public void close() {
        for (var follower : followers) follower.close();
        followers.clear();
        motor.close();
    }
}
