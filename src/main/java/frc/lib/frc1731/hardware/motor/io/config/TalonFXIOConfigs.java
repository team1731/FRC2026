package frc.lib.frc1731.hardware.motor.io.config;

import static edu.wpi.first.units.Units.Rotations;

import java.util.Objects;
import java.util.function.Consumer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.*;

import edu.wpi.first.units.measure.Angle;
import frc.lib.frc1731.PIDGains;
import frc.lib.frc1731.hardware.motor.PortConfig;

/**
 * Mutable, chainable builder for Phoenix 6 Talon FX configurations. Unspecified
 * settings retain Phoenix defaults. This class does not configure hardware.
 *
 * <pre>{@code
 * TalonFXConfiguration config = new TalonFXIOConfigs()
 *     .withCANCoder(new PortConfig("MainCANBus", 14), new CANCoderIOConfig(0.123789, SensorDirectionValue.Clockwise_Positive))
 *     .withPIDGains(20.0, 0.0, 0.2)
 *     .withCurrentLimits(40.0, 80.0)
 *     .withMotionMagic(5.0, 10.0)
 *     .brake()
 *     .invert()
 *     .build();
 * }</pre>
 */
public final class TalonFXIOConfigs implements IMotorIOConfigs {
    private final TalonFXConfiguration config;
    private final java.util.List<MotorGroupConfig.Follower<TalonFXConfiguration>> followers = new java.util.ArrayList<>();
    public CANcoder cancoder;

    public TalonFXIOConfigs() {
        config = new TalonFXConfiguration();
    }

    /** Starts from a copy of an existing configuration. */
    public TalonFXIOConfigs(TalonFXConfiguration initialConfig) {
        config = Objects.requireNonNull(initialConfig).clone();
    }

    /**
     * Uses a remote CANcoder on the motor's CAN bus, preserving configured ratios.
     * Configure the CANcoder's offset and direction separately.
     */
    public TalonFXIOConfigs withCANCoder(PortConfig port, CANCoderIOConfigs coderCfg) {
        this.config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        this.config.Feedback.FeedbackRemoteSensorID = port.kPort;
        cancoder = new CANcoder(port.kPort, port.kBus);
        cancoder.getConfigurator().apply(coderCfg.build());
        return this;
    }

    /** Selects the internal rotor sensor with rotor rotations per mechanism rotation. */
    public TalonFXIOConfigs withRotorFeedback(double gearRatio) {
        requireRatio(gearRatio);
        // config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.FeedbackRemoteSensorID = 0;
        config.Feedback.RotorToSensorRatio = 1.0;
        config.Feedback.SensorToMechanismRatio = gearRatio;
        return this;
    }

    /** Sets sensor rotations per mechanism rotation (not an arbitrary unit conversion). */
    public TalonFXIOConfigs withSensorToMechanismRatio(double ratio) {
        requireRatio(ratio);
        config.Feedback.SensorToMechanismRatio = ratio;
        return this;
    }

    /** Sets rotor rotations per remote sensor rotation, used by fused/synchronized feedback. */
    public TalonFXIOConfigs withRotorToSensorRatio(double ratio) {
        requireRatio(ratio);
        config.Feedback.RotorToSensorRatio = ratio;
        return this;
    }

    /** Sets slot 0 PID gains without changing feedforward or gravity settings. */
    public TalonFXIOConfigs withPIDGains(PIDGains gains) {
        if (gains.getSlot() == 2) {
            config.Slot2.kP = gains.kP;
            config.Slot2.kI = gains.kI;
            config.Slot2.kD = gains.kD;
            config.Slot2.kS = gains.kS;
            config.Slot2.kV = gains.kV;
            config.Slot2.kA = gains.kA;
            config.Slot2.kG = gains.kG;
            config.Slot2.GravityType = gains.kGravityType;
        } else if (gains.getSlot() == 1) {
            config.Slot1.kP = gains.kP;
            config.Slot1.kI = gains.kI;
            config.Slot1.kD = gains.kD;
            config.Slot1.kS = gains.kS;
            config.Slot1.kV = gains.kV;
            config.Slot1.kA = gains.kA;
            config.Slot1.kG = gains.kG;
            config.Slot1.GravityType = gains.kGravityType;
        } else {
            config.Slot0.kP = gains.kP;
            config.Slot0.kI = gains.kI;
            config.Slot0.kD = gains.kD;
            config.Slot0.kS = gains.kS;
            config.Slot0.kV = gains.kV;
            config.Slot0.kA = gains.kA;
            config.Slot0.kG = gains.kG;
            config.Slot0.GravityType = gains.kGravityType;
        }
        
        config.ClosedLoopGeneral.ContinuousWrap = gains.continuousInput;

        return this;
    }

    /** Sets clockwise-positive output. Repeated calls do not toggle direction. */
    public TalonFXIOConfigs invert() {
        return invert(true);
    }

    /** True means clockwise-positive; false means counterclockwise-positive. */
    public TalonFXIOConfigs invert(boolean inverted) {
        config.MotorOutput.Inverted = inverted
                ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        return this;
    }

    /** Sets the neutral behavior used when the motor is not actively driven. */
    public TalonFXIOConfigs withNeutralMode(NeutralModeValue mode) {
        config.MotorOutput.NeutralMode = Objects.requireNonNull(mode);
        return this;
    }

    /** Configures the motor to brake when output is neutral. */
    public TalonFXIOConfigs brake() {
        return withNeutralMode(NeutralModeValue.Brake);
    }

    /** Configures the motor to coast when output is neutral. */
    public TalonFXIOConfigs coast() {
        return withNeutralMode(NeutralModeValue.Coast);
    }

    /** Sets and enables both current limits, in amps. */
    public TalonFXIOConfigs withCurrentLimits(double supplyAmps, double statorAmps) {
        config.CurrentLimits.SupplyCurrentLimit = supplyAmps;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = statorAmps;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        return this;
    }

    /** Sets and enables stator current limit, in amps. */
    public TalonFXIOConfigs withCurrentLimits(double statorAmps) {
        config.CurrentLimits.StatorCurrentLimit = statorAmps;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        return this;
    }

    /** Sets cruise velocity (mechanism rotations/s) and acceleration (rotations/s^2), with zero jerk. */
    public TalonFXIOConfigs withMotionMagicSpeeds(double cruiseVelocity, double acceleration) {
        return withMotionMagicSpeeds(cruiseVelocity, acceleration, 0.0);
    }

    /** Sets Motion Magic constraints in mechanism rotations/s, rotations/s^2, and rotations/s^3. */
    public TalonFXIOConfigs withMotionMagicSpeeds(double cruiseVelocity, double acceleration, double jerk) {
        config.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
        config.MotionMagic.MotionMagicAcceleration = acceleration;
        config.MotionMagic.MotionMagicJerk = jerk;
        return this;
    }

    /** Enables reverse and forward soft limits, in mechanism rotations. */
    public TalonFXIOConfigs withSoftLimits(double reverseRotations, double forwardRotations) {
        if (!Double.isFinite(reverseRotations) || !Double.isFinite(forwardRotations)
                || reverseRotations > forwardRotations) {
            throw new IllegalArgumentException("Soft limits must be finite with reverse <= forward");
        }
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseRotations;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardRotations;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        return this;
    }

    /** Enables reverse and forward soft limits, in mechanism rotations. */
    public TalonFXIOConfigs withSoftLimits(Angle reverse, Angle forward) {
        double reverseRotations = reverse.in(Rotations);
        double forwardRotations = forward.in(Rotations);
        if (!Double.isFinite(reverseRotations) || !Double.isFinite(forwardRotations)
                || reverseRotations > forwardRotations) {
            throw new IllegalArgumentException("Soft limits must be finite with reverse <= forward");
        }
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseRotations;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardRotations;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        return this;
    }

    /** Enables/disables position wrapping for mechanisms that rotate continuously. */
    public TalonFXIOConfigs withContinuousWrap(boolean enabled) {
        config.ClosedLoopGeneral.ContinuousWrap = enabled;
        return this;
    }

    /**
     * Customizes any Phoenix setting, including other gain slots or feedback modes.
     * Changes are applied immediately, so later builder calls take precedence.
     */
    public TalonFXIOConfigs configure(Consumer<TalonFXConfiguration> customizer) {
        Objects.requireNonNull(customizer).accept(config);
        return this;
    }

    /** Returns an independent snapshot suitable for MotorIOTalonFX factory methods. */
    public TalonFXConfiguration build() {
        if (!followers.isEmpty()) {
            throw new IllegalStateException("Pass this IOConfigs builder directly to the MotorIO factory to retain followers");
        }
        return config.clone();
    }

    /**
     * Adds a same-family follower on the leader's bus. Settings are copied now.
     * Supply follower current limits, neutral mode, and (FXS) motor arrangement explicitly.
     * Do not copy leader feedback/position limits unless valid for this follower's own encoder.
     */
    public TalonFXIOConfigs withFollower(int deviceId, boolean inverted, TalonFXConfiguration followerConfig) {
        MotorGroupConfig.requireDeviceId(deviceId);
        Objects.requireNonNull(followerConfig);
        if (followers.stream().anyMatch(f -> f.deviceId() == deviceId)) {
            throw new IllegalArgumentException("Duplicate follower CAN ID: " + deviceId);
        }
        followers.add(new MotorGroupConfig.Follower<>(deviceId, inverted, followerConfig.clone()));
        return this;
    }

    /** Adds an aligned follower that inherits the leader's final configuration. */
    public TalonFXIOConfigs withFollower(int deviceId) {
        return withFollower(deviceId, false);
    }

    /**
     * Inherits all leader settings when the MotorIO is constructed, including later builder edits.
     * Each follower receives an independent copy. Inversion is relative to leader output.
     * Use the explicit-config overload when follower feedback or limits must differ.
     */
    public TalonFXIOConfigs withFollower(int deviceId, boolean inverted) {
        MotorGroupConfig.requireDeviceId(deviceId);
        if (followers.stream().anyMatch(f -> f.deviceId() == deviceId)) {
            throw new IllegalArgumentException("Duplicate follower CAN ID: " + deviceId);
        }
        // Retain the leader config until buildMotorGroup snapshots each controller separately.
        followers.add(new MotorGroupConfig.Follower<>(deviceId, inverted, config));
        return this;
    }

    /** Adds an aligned follower with its own controller settings. */
    public TalonFXIOConfigs withFollower(int deviceId, TalonFXConfiguration followerConfig) {
        return withFollower(deviceId, false, followerConfig);
    }

    /** Hardware-free snapshot used by MotorIO factories; validates before opening controllers. */
    public MotorGroupConfig<TalonFXConfiguration> buildMotorGroup(int leaderId) {
        MotorGroupConfig.requireDeviceId(leaderId);
        if (followers.stream().anyMatch(f -> f.deviceId() == leaderId)) {
            throw new IllegalArgumentException("A motor cannot follow itself: " + leaderId);
        }
        return new MotorGroupConfig<>(config.clone(), followers.stream()
            .map(f -> new MotorGroupConfig.Follower<>(f.deviceId(), f.inverted(), f.config().clone()))
            .toList());
    }

    private static void requireRatio(double ratio) {
        if (!Double.isFinite(ratio) || ratio == 0.0 || Math.abs(ratio) > 1000.0) {
            throw new IllegalArgumentException("Feedback ratio must be finite, nonzero, and within [-1000, 1000]");
        }
    }
}
