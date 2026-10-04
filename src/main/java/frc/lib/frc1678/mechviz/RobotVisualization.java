package frc.lib.frc1678.mechviz;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.lib.frc1678.mechviz.mechanism.mech.PivotingMechanism3d;

/**
 * Supplier-driven geometry model for real or simulated robots. All access is on the robot
 * scheduler thread. Pose reads return the last completed loop; they do not poll suppliers.
 * Mechanism poses are robot-relative, while Robot is field-relative. This models geometry,
 * not physics, sensor validity, command targets, or actuator control.
 */
public final class RobotVisualization extends RobotVisualizer implements AutoCloseable {
    private final String name;
    private final Supplier<Pose2d> robotPoseSource;
    private final Map<String, Supplier<Pose3d>> mechanisms;
    private final Consumer<Map<String, Pose3d>> output;
    private final Map<String, StructPublisher<Pose3d>> publishers = new LinkedHashMap<>();
    private Map<String, Pose3d> poses = Map.of();
    private boolean publishingEnabled = true;
    private boolean closed;

    private RobotVisualization(Builder builder) {
        name = builder.name;
        robotPoseSource = builder.robotPose;
        mechanisms = new LinkedHashMap<>(builder.mechanisms);
        output = builder.output;
    }

    /** Builder configurations are already registered; init() is harmless and repeatable. */
    @Override public void registerRobots() {}

    /** Enable/disable output only. Local pose calculations continue on every loop. */
    public void setPublishingEnabled(boolean enabled) { publishingEnabled = enabled; }

    /** Immutable latest snapshot; empty until the first successful loop. */
    public Map<String, Pose3d> getPoses() { return poses; }

    /** Get a registered component's last pose, rejecting absent or not-yet-sampled names. */
    public Pose3d getPose(String key) {
        Pose3d pose = poses.get(key);
        if (pose == null) throw new IllegalStateException("No sampled pose: " + key);
        return pose;
    }

    /** Compose a robot-relative mechanism with the cached field-relative robot pose. */
    public Pose3d getFieldPose(String mechanism) {
        return getPose("Robot").transformBy(new Transform3d(Pose3d.kZero, getPose(mechanism)));
    }

    @Override public void loop() {
        if (closed) return;
        Map<String, Pose3d> next = new LinkedHashMap<>();
        next.put("Robot", new Pose3d(Objects.requireNonNull(robotPoseSource.get())));
        mechanisms.forEach((key, source) -> next.put(key, Objects.requireNonNull(source.get())));
        poses = java.util.Collections.unmodifiableMap(next);
        if (!publishingEnabled) return;
        if (output != null) {
            output.accept(poses);
        } else {
            poses.forEach((key, pose) -> publishers.computeIfAbsent(key, k ->
                    NetworkTableInstance.getDefault().getTable("RobotVisualization")
                        .getSubTable(name).getStructTopic(k, Pose3d.struct).publish()).set(pose));
        }
    }

    @Override public void close() {
        publishers.values().forEach(StructPublisher::close);
        publishers.clear();
        closed = true;
    }

    public static final class Builder {
        private final String name;
        private Supplier<Pose2d> robotPose = Pose2d::new;
        private final Map<String, Supplier<Pose3d>> mechanisms = new LinkedHashMap<>();
        private Consumer<Map<String, Pose3d>> output;

        Builder(String name) {
            if (name == null || name.isBlank()) throw new IllegalArgumentException("Robot name required");
            this.name = name;
        }

        public Builder withRobotPose(Supplier<Pose2d> source) {
            robotPose = Objects.requireNonNull(source);
            return this;
        }

        /** Arbitrary robot-relative pose, including linked mechanisms or existing 1678 models. */
        public Builder withMechanism(String key, Supplier<Pose3d> source) {
            if (key == null || key.isBlank() || key.equals("Robot") || mechanisms.containsKey(key))
                throw new IllegalArgumentException("Unique mechanism name required: " + key);
            mechanisms.put(key, Objects.requireNonNull(source));
            return this;
        }

        /** Local rotation about the mount's Y axis. Positive input raises the arm toward +Z. */
        public Builder withArm(String key, Pose3d mount, DoubleSupplier radians) {
            Objects.requireNonNull(mount);
            Objects.requireNonNull(radians);
            var model = new PivotingMechanism3d(mount, () -> new Transform3d(
                    Translation3d.kZero, new Rotation3d(0, -radians.getAsDouble(), 0)));
            return withMechanism(key, () -> { model.loop(); return model.getPivotedPose(); });
        }

        /** Translation along the mounting frame's local +Z axis, in meters. */
        public Builder withElevator(String key, Pose3d mount, DoubleSupplier meters) {
            Objects.requireNonNull(mount);
            Objects.requireNonNull(meters);
            var model = new PivotingMechanism3d(mount, () -> new Transform3d(
                    new Translation3d(0, 0, meters.getAsDouble()), Rotation3d.kZero));
            return withMechanism(key, () -> { model.loop(); return model.getPivotedPose(); });
        }

        /** Replace NetworkTables output, e.g. with AdvantageKit or a test/preview recorder. */
        public Builder withOutput(Consumer<Map<String, Pose3d>> sink) {
            output = Objects.requireNonNull(sink);
            return this;
        }

        /** Does not sample suppliers, instantiate hardware, or open publishers. */
        public RobotVisualization build() { return new RobotVisualization(this); }
    }
}
