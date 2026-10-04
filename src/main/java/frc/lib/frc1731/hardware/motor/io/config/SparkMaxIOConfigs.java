package frc.lib.frc1731.hardware.motor.io.config;

import java.util.Objects;
import java.util.function.Consumer;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import frc.lib.frc1731.PIDGains;

/**
 * Mutable, chainable REV SPARK Max configuration builder; does not configure hardware.
 * Unspecified settings follow REV's configure/reset behavior.
 * withRotorFeedback uses mechanism rotations for position and mechanism RPM for velocity.
 *
 * <pre>{@code
 * var config = new SparkMaxIOConfigs()
 *     .withRotorFeedback(10.0)
 *     .withPIDGains(new PIDGains().setPID(0.1, 0.0, 0.0))
 *     .withCurrentLimits(40)
 *     .withMAXMotionSpeeds(300.0, 600.0)
 *     .brake().invert().build();
 * }</pre>
 */
public final class SparkMaxIOConfigs implements IMotorIOConfigs {
    private final SparkMaxConfig config;
    private final java.util.List<MotorGroupConfig.Follower<SparkMaxConfig>> followers = new java.util.ArrayList<>();

    public SparkMaxIOConfigs() {
        config = new SparkMaxConfig();
    }

    /** Starts from a copy of an existing configuration. */
    public SparkMaxIOConfigs(SparkMaxConfig initialConfig) {
        config = new SparkMaxConfig().apply(Objects.requireNonNull(initialConfig));
    }

    /** Selects the primary encoder and sets rotor rotations per mechanism rotation. */
    public SparkMaxIOConfigs withRotorFeedback(double gearRatio) {
        withSensorToMechanismRatio(gearRatio);
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        return this;
    }

    /** Sets primary encoder gearing: mechanism rotations and RPM; preserves feedback selection. */
    public SparkMaxIOConfigs withSensorToMechanismRatio(double ratio) {
        if (!Double.isFinite(ratio) || ratio <= 0.0) {
            throw new IllegalArgumentException("Encoder ratio must be finite and positive");
        }
        config.encoder.positionConversionFactor(1.0 / ratio);
        config.encoder.velocityConversionFactor(1.0 / ratio);
        return this;
    }

    /**
     * Sets PID, I-zone, feedforward, and tolerance in slot 0-3.
     * Gains use REV units, including volts per mechanism RPM for kV.
     * Arm gravity uses kCos; zero the encoder at horizontal. Position must be
     * mechanism rotations, or customize feedForward.kCosRatio separately.
     * Wrapping is global. Soft limits are applied when requested by the gains.
     */
    public SparkMaxIOConfigs withPIDGains(PIDGains gains) {
        Objects.requireNonNull(gains);
        ClosedLoopSlot slot = slotFor(gains.getSlot());
        config.closedLoop.pid(gains.kP, gains.kI, gains.kD, slot)
                .iZone(gains.kIZone, slot).allowedClosedLoopError(gains.tolerance, slot);
        config.closedLoop.feedForward
                .kS(gains.kS, slot).kV(gains.kV, slot).kA(gains.kA, slot)
                .kG(gains.kGravityType == GravityTypeValue.Elevator_Static ? gains.kG : 0.0, slot)
                .kCos(gains.kGravityType == GravityTypeValue.Arm_Cosine ? gains.kG : 0.0, slot);
        if (gains.continuousInput) {
            withContinuousWrap(true, gains.continuousMin, gains.continuousMax);
        } else {
            withContinuousWrap(false);
        }
        if (gains.softLimit) {
            withSoftLimits(gains.softLimitMin, gains.softLimitMax);
        }
        return this;
    }

    /** Sets inversion; repeated calls do not toggle direction. */
    public SparkMaxIOConfigs invert() {
        return invert(true);
    }

    public SparkMaxIOConfigs invert(boolean inverted) {
        config.inverted(inverted);
        return this;
    }

    public SparkMaxIOConfigs withNeutralMode(IdleMode mode) {
        config.idleMode(Objects.requireNonNull(mode));
        return this;
    }

    public SparkMaxIOConfigs brake() {
        return withNeutralMode(IdleMode.kBrake);
    }

    public SparkMaxIOConfigs coast() {
        return withNeutralMode(IdleMode.kCoast);
    }

    /** Sets REV's smart motor current limit in amps, not a CTRE supply limit. */
    public SparkMaxIOConfigs withCurrentLimits(int smartLimitAmps) {
        config.smartCurrentLimit(smartLimitAmps);
        return this;
    }

    /** Sets smart motor and secondary chopping current limits, in amps. */
    public SparkMaxIOConfigs withCurrentLimits(int smartLimitAmps, double secondaryLimitAmps) {
        config.smartCurrentLimit(smartLimitAmps);
        config.secondaryCurrentLimit(secondaryLimitAmps);
        return this;
    }

    /** Sets nominal voltage compensation in volts. */
    public SparkMaxIOConfigs withVoltageCompensation(double nominalVoltage) {
        config.voltageCompensation(nominalVoltage);
        return this;
    }

    /** Sets slot 0 MAXMotion cruise velocity (mechanism RPM) and acceleration (RPM/s). */
    public SparkMaxIOConfigs withMAXMotionSpeeds(double cruiseVelocity, double acceleration) {
        return withMAXMotionSpeeds(cruiseVelocity, acceleration, 0);
    }

    /** Sets trapezoidal MAXMotion constraints in mechanism RPM and RPM/s for slot 0-3. */
    public SparkMaxIOConfigs withMAXMotionSpeeds(double cruiseVelocity, double acceleration, int slot) {
        ClosedLoopSlot closedLoopSlot = slotFor(slot);
        config.closedLoop.maxMotion.cruiseVelocity(cruiseVelocity, closedLoopSlot)
                .maxAcceleration(acceleration, closedLoopSlot)
                .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, closedLoopSlot);
        return this;
    }

    /** Enables reverse and forward soft limits in converted encoder position units. */
    public SparkMaxIOConfigs withSoftLimits(double reverseRotations, double forwardRotations) {
        if (!Double.isFinite(reverseRotations) || !Double.isFinite(forwardRotations)
                || reverseRotations > forwardRotations) {
            throw new IllegalArgumentException("Soft limits must be finite with reverse <= forward");
        }
        config.softLimit.reverseSoftLimit(reverseRotations).forwardSoftLimit(forwardRotations)
                .reverseSoftLimitEnabled(true).forwardSoftLimitEnabled(true);
        return this;
    }

    /** Enables/disables wrapping over one mechanism rotation (0 to 1). */
    public SparkMaxIOConfigs withContinuousWrap(boolean enabled) {
        return withContinuousWrap(enabled, 0.0, 1.0);
    }

    /** Sets a wrapping range in converted encoder position units. */
    public SparkMaxIOConfigs withContinuousWrap(boolean enabled, double min, double max) {
        if (!Double.isFinite(min) || !Double.isFinite(max) || min >= max) {
            throw new IllegalArgumentException("Wrapping range must be finite with min < max");
        }
        config.closedLoop.positionWrappingEnabled(enabled).positionWrappingInputRange(min, max);
        return this;
    }

    /** Customizes any REV setting immediately; later builder calls take precedence. */
    public SparkMaxIOConfigs configure(Consumer<SparkMaxConfig> customizer) {
        Objects.requireNonNull(customizer).accept(config);
        return this;
    }

    /** Returns an independent snapshot suitable for MotorIOSparkMax factory methods. */
    public SparkMaxConfig build() {
        if (!followers.isEmpty()) {
            throw new IllegalStateException("Pass this IOConfigs builder directly to the MotorIO factory to retain followers");
        }
        return new SparkMaxConfig().apply(config);
    }

    /**
     * Adds a same-family follower on the leader's bus. Settings are copied now.
     * Supply follower current limits, neutral mode, and (FXS) motor arrangement explicitly.
     * Do not copy leader feedback/position limits unless valid for this follower's own encoder.
     */
    public SparkMaxIOConfigs withFollower(int deviceId, boolean inverted, SparkMaxConfig followerConfig) {
        MotorGroupConfig.requireDeviceId(deviceId);
        Objects.requireNonNull(followerConfig);
        if (followers.stream().anyMatch(f -> f.deviceId() == deviceId)) {
            throw new IllegalArgumentException("Duplicate follower CAN ID: " + deviceId);
        }
        followers.add(new MotorGroupConfig.Follower<>(deviceId, inverted, new SparkMaxConfig().apply(followerConfig)));
        return this;
    }

    /** Adds an aligned follower that inherits the leader's final configuration. */
    public SparkMaxIOConfigs withFollower(int deviceId) {
        return withFollower(deviceId, false);
    }

    /**
     * Inherits all leader settings when the MotorIO is constructed, including later builder edits.
     * Each follower receives an independent copy. Inversion is relative to leader output.
     * Use the explicit-config overload when follower feedback or limits must differ.
     */
    public SparkMaxIOConfigs withFollower(int deviceId, boolean inverted) {
        MotorGroupConfig.requireDeviceId(deviceId);
        if (followers.stream().anyMatch(f -> f.deviceId() == deviceId)) {
            throw new IllegalArgumentException("Duplicate follower CAN ID: " + deviceId);
        }
        // Retain the leader config until buildMotorGroup snapshots each controller separately.
        followers.add(new MotorGroupConfig.Follower<>(deviceId, inverted, config));
        return this;
    }

    /** Adds an aligned follower with its own controller settings. */
    public SparkMaxIOConfigs withFollower(int deviceId, SparkMaxConfig followerConfig) {
        return withFollower(deviceId, false, followerConfig);
    }

    /** Hardware-free snapshot used by MotorIO factories; validates before opening controllers. */
    public MotorGroupConfig<SparkMaxConfig> buildMotorGroup(int leaderId) {
        MotorGroupConfig.requireDeviceId(leaderId);
        if (followers.stream().anyMatch(f -> f.deviceId() == leaderId)) {
            throw new IllegalArgumentException("A motor cannot follow itself: " + leaderId);
        }
        return new MotorGroupConfig<>(new SparkMaxConfig().apply(config), followers.stream()
            .map(f -> new MotorGroupConfig.Follower<>(f.deviceId(), f.inverted(), new SparkMaxConfig().apply(f.config())))
            .toList());
    }

    private static ClosedLoopSlot slotFor(int slot) {
        return switch (slot) {
            case 0 -> ClosedLoopSlot.kSlot0;
            case 1 -> ClosedLoopSlot.kSlot1;
            case 2 -> ClosedLoopSlot.kSlot2;
            case 3 -> ClosedLoopSlot.kSlot3;
            default -> throw new IllegalArgumentException("SPARK PID slot must be between 0 and 3");
        };
    }
}
