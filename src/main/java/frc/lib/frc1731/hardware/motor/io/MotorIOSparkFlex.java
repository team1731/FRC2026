package frc.lib.frc1731.hardware.motor.io;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.frc1678.sim.MechanismSim;
import frc.lib.frc1731.hardware.motor.*;
import frc.robot.Robot;
import frc.lib.frc1731.hardware.motor.io.config.SparkFlexIOConfigs;
import frc.lib.frc1731.hardware.motor.io.config.MotorGroupConfig;

/**
 * {@link MotorIO} implementation for REV SPARK Flex brushless motor controllers.
 */
public class MotorIOSparkFlex extends MotorIO implements AutoCloseable {
    private final java.util.List<SparkFlex> followers = new java.util.ArrayList<>();
    private SparkFlex motor;
    private SparkClosedLoopController motorCtrl;
    private RelativeEncoder encoder;
    private SparkFlexConfig config;
    private SparkSim motorSim;

    private MotorIOSparkFlex(MotorConstants constants, PortConfig port, SparkFlexConfig config) {
        super(constants, port);
        this.motor = new SparkFlex(port.kPort, MotorType.kBrushless);
        this.config = config;
        this.motorCtrl = motor.getClosedLoopController();
        this.encoder = motor.getEncoder();

        this.motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Creates a NEO SPARK Flex wrapper with a custom REV configuration.
     *
     * @param port CAN bus and ID for the motor
     * @param config SPARK Flex configuration to apply on construction
     * @return configured SPARK Flex motor IO
     */
    public static MotorIOSparkFlex generateNeo(PortConfig port, SparkFlexConfig config) {
        return new MotorIOSparkFlex(MotorConstants.kNeo, port, config);
    }

    /** Creates a leader and configured followers; pass the builder without calling build(). */
    public static MotorIOSparkFlex generateNeo(PortConfig port, SparkFlexIOConfigs configs) {
        var group = configs.buildMotorGroup(port.kPort);
        var io = new MotorIOSparkFlex(MotorConstants.kNeo, port, group.leader());
        try {
            io.configureFollowers(group);
            return io;
        } catch (RuntimeException failure) {
            io.close();
            throw failure;
        }
    }

    /**
     * Creates a NEO SPARK Flex wrapper with default REV configuration.
     *
     * @param port CAN bus and ID for the motor
     * @return configured SPARK Flex motor IO
     */
    public static MotorIOSparkFlex generateNeo(PortConfig port) {
        return new MotorIOSparkFlex(MotorConstants.kNeo, port, new SparkFlexConfig());
    }

    /**
     * Creates a NEO 550 SPARK Flex wrapper with a custom REV configuration.
     */
    public static MotorIOSparkFlex generateNeo550(PortConfig port, SparkFlexConfig config) {
        return new MotorIOSparkFlex(MotorConstants.kNeo550, port, config);
    }

    /** Creates a leader and configured followers; pass the builder without calling build(). */
    public static MotorIOSparkFlex generateNeo550(PortConfig port, SparkFlexIOConfigs configs) {
        var group = configs.buildMotorGroup(port.kPort);
        var io = new MotorIOSparkFlex(MotorConstants.kNeo550, port, group.leader());
        try {
            io.configureFollowers(group);
            return io;
        } catch (RuntimeException failure) {
            io.close();
            throw failure;
        }
    }

    /**
     * Creates a NEO 550 SPARK Flex wrapper with default REV configuration.
     */
    public static MotorIOSparkFlex generateNeo550(PortConfig port) {
        return new MotorIOSparkFlex(MotorConstants.kNeo550, port, new SparkFlexConfig());
    }

    @Override
    @SuppressWarnings("unchecked")
    public SparkFlex getMotor() {
        return this.motor;
    }

    @Override
    @SuppressWarnings("unchecked")
    public MotorIOSparkFlex withSimulation(MechanismSim sim) {
        this.mechSim = sim;
        this.motorSim = new SparkSim(motor, sim.getMotor());
        return this;
    }

    @Override
    public void resetEncoderPosition(Angle position) {
        this.encoder.setPosition(position.in(Rotations));
        this.mechSim.setState(position, RotationsPerSecond.of(0));
    }

    @Override
    public void updateTrapezoidalSpeeds(AngularVelocity vel, AngularAcceleration accel, int slot) {
        this.config.closedLoop.maxMotion
            .cruiseVelocity(vel.in(RPM), getSlot(slot))
            .maxAcceleration(accel.in(RPM.per(Second)), getSlot(slot))
            .positionMode(com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode.kMAXMotionTrapezoidal, getSlot(slot))
        ;

        this.motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void coast() {
        this.config.idleMode(IdleMode.kCoast);
        this.motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        this.setPercent(0.0);
    }

    @Override
    public void brake() {
        this.config.idleMode(IdleMode.kBrake);
        this.motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        this.setPercent(0.0);
    }

    @Override
    public void setPercent(double percent) {
        this.motor.set(percent);
    }

    @Override
    public void setVoltage(double volts) {
        this.motor.setVoltage(volts);
    }

    @Override
    public void setPosition(Angle position, int slot) {
        this.motorCtrl.setSetpoint(position.in(Rotations), ControlType.kPosition, getSlot(slot));
    }

    @Override
    public void setPositionTrapezoidal(Angle position, int slot) {
        this.motorCtrl.setSetpoint(position.in(Rotations), ControlType.kMAXMotionPositionControl, getSlot(slot));
    }

    @Override
    public void setVelocity(AngularVelocity velocity, int slot) {
        this.motorCtrl.setSetpoint(velocity.in(RPM), ControlType.kVelocity, getSlot(slot));
    }

    @Override
    public Angle getPosition() {
        return Rotations.of(this.encoder.getPosition());
    }

    @Override
    public AngularVelocity getVelocity() {
        return RPM.of(this.encoder.getVelocity());
    }

    @Override
    public double getAppliedDutyCycle() {
        return motor.getAppliedOutput();
    }

    @Override
    public double getAppliedVoltage() {
        return this.motor.getAppliedOutput() * motor.getBusVoltage(); // Experimental and may not work?
    }

    @Override
    public Angle getForwardLimit() {
        return Rotations.of(this.motor.configAccessor.limitSwitch.getForwardLimitSwitchPosition());
    }

    @Override
    public Angle getReverseLimit() {
        return Rotations.of(this.motor.configAccessor.limitSwitch.getReverseLimitSwitchPosition());
    }

    @Override
    public void simPeriodic() {
        // 1. Only move forward if we are in simulation mode and mechanism sim is initialized
        if (!Robot.isSimulation() || mechSim == null || motorSim == null) return;

        // 2. Supply the current battery sag voltage to the simulated SPARK Flex bus
        double batteryVoltage = RobotController.getBatteryVoltage();
        motorSim.setBusVoltage(batteryVoltage);

        // 3. Calculate whatever voltage the SPARK Flex's control loop decided to apply.
        // (getAppliedOutput() returns the duty cycle [-1.0 to 1.0])
        double appliedVoltage = motorSim.getAppliedOutput() * motorSim.getBusVoltage();
        mechSim.setVoltage(Volts.of(appliedVoltage));

        // 4. Advance the WPILib physics simulation plant
        mechSim.simulate();

        // 5. Convert the resulting mechanism velocity back into motor-space RPM.
        // (WPILib physics models standardly output in radians/second)
        double mechanismRadPerSec = mechSim.getVelocity().in(RadiansPerSecond);
        
        // mechanismToRotor() applies the gearing ratio configured on your sim plant.
        // Multiplying by 9.5493 (or standard WPILib Units conversion) translates Rad/Sec to RPM.
        double motorRPM = mechSim.mechanismToRotor(RotationsPerSecond.of(mechanismRadPerSec / (2 * Math.PI)))
                            .in(RotationsPerSecond) * 60.0;

        // 6. Feed the physics step into the SPARK Flex. 
        // This automatically recalculates internal position accumulation and filters values!
        motorSim.iterate(
            motorRPM,          // Velocity in RPM (Motor-space)
            batteryVoltage,    // Input Bus Voltage
            0.020              // Timestep loop duration (Standard 20ms)
        );
    }

    private ClosedLoopSlot getSlot(int slot) {
        ClosedLoopSlot clSlot;
        if (slot == 1) {
            clSlot = ClosedLoopSlot.kSlot1;
        } else if (slot == 2) {
            clSlot = ClosedLoopSlot.kSlot2;
        } else if (slot == 3) {
            clSlot = ClosedLoopSlot.kSlot3;
        } else {
            clSlot = ClosedLoopSlot.kSlot0;
        }
        return clSlot;
    }

    private void configureFollowers(MotorGroupConfig<SparkFlexConfig> group) {
        for (var spec : group.followers()) {
            var follower = new SparkFlex(spec.deviceId(), MotorType.kBrushless);
            followers.add(follower);
            spec.config().follow(motor, spec.inverted());
            var status = follower.configure(spec.config(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            if (status != REVLibError.kOk) throw new IllegalStateException("Follower configuration failed: " + status);
        }
    }

    /** Retained controllers for diagnostics. Direct control can break hardware following. */
    public java.util.List<SparkFlex> getFollowerMotors() {
        return java.util.List.copyOf(followers);
    }

    /** Releases the leader and followers when this IO is no longer used. */
    @Override public void close() {
        for (var follower : followers) follower.close();
        followers.clear();
        motor.close();
    }
}
