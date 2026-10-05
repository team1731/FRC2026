package frc.lib.frc1731.subsystem;

import static edu.wpi.first.units.Units.Radians;

import java.util.Objects;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc1731.hardware.motor.io.MotorIO;

/**
 * Position-controlled, vertical-axis turret that aims at field-relative XY targets.
 * The robot pose and target must use the same field frame; alliance flipping is the caller's job.
 * Motor feedback must represent mechanism angle, positive counterclockwise, with zero along
 * the turret mount's +X axis. Configure gearing and hardware limits in the motor IO.
 *
 * @param <IO> motor IO implementation used by the turret
 */
public abstract class BaseTurretSubsystem<IO extends MotorIO> extends BaseServoSubsystem<IO> {
    private final Supplier<Pose2d> robotPose;
    private final Transform3d robotToTurret;

    /**
     * @param motor turret motor IO
     * @param robotPose live field-relative robot pose
     * @param robotToTurret fixed transform to the turret pivot at encoder angle zero;
     *        translation and yaw determine planar aiming, height is ignored
     */
    public BaseTurretSubsystem(IO motor, Supplier<Pose2d> robotPose, Transform3d robotToTurret) {
        super(motor);
        this.robotPose = Objects.requireNonNull(robotPose, "robotPose");
        this.robotToTurret = Objects.requireNonNull(robotToTurret, "robotToTurret");
    }

    /** Field-relative pose of the fixed turret mount (does not include motor rotation). */
    public Pose2d getTurretMountPose() {
        return new Pose3d(robotPose.get()).transformBy(robotToTurret).toPose2d();
    }

    /**
     * Computes the target angle relative to the mount's zero heading, in [-pi, pi].
     * Returns the current motor angle if the target coincides with the turret pivot.
     * This does not select a multi-turn route or avoid mechanical limits.
     */
    public Angle getTrackingAngle(Translation2d target) {
        Pose2d mount = getTurretMountPose();
        Translation2d delta = Objects.requireNonNull(target, "target").minus(mount.getTranslation());
        if (delta.getNorm() < 1e-9) {
            return getPosition();
        }
        return Radians.of(calculateTrackingHeading(mount, target).getRadians());
    }

    /** Pure planar aiming calculation; callers must handle a target at the pivot. */
    static Rotation2d calculateTrackingHeading(Pose2d mount, Translation2d target) {
        return target.minus(mount.getTranslation()).getAngle().minus(mount.getRotation());
    }

    /** Aims at a fixed field point until interrupted, recalculating as the robot moves. */
    public Command track(Translation2d target) {
        Objects.requireNonNull(target, "target");
        return trackTranslation(() -> target);
    }

    /** Aims at the pose's translation; its rotation is ignored. */
    public Command track(Pose2d target) {
        return track(Objects.requireNonNull(target, "target").getTranslation());
    }

    /** Samples both the target pose and robot pose each scheduler cycle. */
    public Command track(Supplier<Pose2d> target) {
        Objects.requireNonNull(target, "target");
        return trackTranslation(() -> target.get().getTranslation());
    }

    /** Named separately because Java erases the generic type of Supplier overloads. */
    public Command trackTranslation(Supplier<Translation2d> target) {
        Objects.requireNonNull(target, "target");
        return setPosition(() -> getTrackingAngle(target.get())).withName("TrackTarget");
    }

    @AutoLog
    public static class BaseTurretInputs {
        public Angle currentPosition, setpointPosition;
        public Pose2d trackedTarget;
        public boolean atSetpoint;
    }
}
