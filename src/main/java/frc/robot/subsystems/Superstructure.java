package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.frc1731.field.FieldPositions;
import frc.robot.Robot;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.indexer.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.shooter.flywheel.*;
import frc.robot.subsystems.shooter.hood.*;
import frc.robot.subsystems.shooter.turret.*;
public class Superstructure extends SubsystemBase {
    private SwerveSubsystem swerve;
    private FlywheelSubsystem leftFlywheel, rightFlywheel;
    private TurretSubsystem leftTurret, rightTurret;
    private HoodSubsystem leftHood, rightHood;
    private IndexerSubsystem indexer;
    private IntakePivotSubsystem pivot;
    private IntakeRollerSubsystem intake;

    private final ShotTable shotTable = ShotTable.getScoringTable();

    // -------------------------------------------------------------------------
    // Target suppliers
    // -------------------------------------------------------------------------

    private final Supplier<Translation2d> kHubSupplier = () -> Robot.isRedAlliance()
        ? new Translation2d(11.915394, 4.034536)
        : new Translation2d(4.625594, 4.034536);

    private final Supplier<Translation2d> kPassSupplier = () -> {
        double x = Robot.isRedAlliance() ? FieldPositions.kFieldLength - 2 : 2;
        double y = Robot.isRedAlliance() && swerve.getCurrentPose().getY() > FieldPositions.kFieldWidth / 2.0 ? FieldPositions.kFieldWidth - 1 : 1;
        return new Translation2d(x, y);
    };

    private Supplier<Translation2d> targetSupplier = kHubSupplier;

    // -------------------------------------------------------------------------
    // Per-turret compensated aim points (written by periodic, read by commands)
    // -------------------------------------------------------------------------

    private Translation2d compensatedLeftTarget  = new Translation2d();
    private Translation2d compensatedRightTarget = new Translation2d();

    // final lambdas — never reassigned, always read the latest field value
    public final Supplier<Translation2d> appliedLeftTargetSupplier  = () -> compensatedLeftTarget;
    public final Supplier<Translation2d> appliedRightTargetSupplier = () -> compensatedRightTarget;
    // Convenience alias kept for callers that only need one (uses left turret)
    public final Supplier<Translation2d> appliedTargetSupplier = appliedLeftTargetSupplier;

    // -------------------------------------------------------------------------
    // Shot parameter targets (written by periodic, read by commands via lambdas)
    // -------------------------------------------------------------------------

    private double targetLeftHood = 0;
    private double targetRightHood = 0;
    private double targetLeftFlywheel = 0;
    private double targetRightFlywheel = 0;

    private boolean adjustTargetForMovingShots = false;

    // -------------------------------------------------------------------------
    // Velocity / acceleration state for the robot chassis
    //
    // We maintain a running derivative of field-relative chassis speeds so we
    // can predict where the turret pivot will be at time-of-flight, not just
    // where it is right now.
    //
    // Layout:
    //   prevVelocity        — field-relative velocity one loop ago (m/s)
    //   chassisAcceleration — finite-difference derivative (m/s²)
    //   prevOmega           — angular velocity one loop ago (rad/s)
    //   angularAcceleration — derivative of omega (rad/s²)
    //
    // We use a small exponential filter (alpha = 0.3) on both acceleration terms
    // to suppress quantisation noise from the 50 Hz odometry update.
    // Lower alpha = smoother but laggier; 0.3 is a good default for FRC.
    // -------------------------------------------------------------------------

    private static final double kDt = 0.02;  // 50 Hz robot loop period (s)
    private static final double kAccelAlpha = 0.3;   // EMA coefficient for accel filter
    private static final double kMaxPredictTof = 1.2;   // clamp TOF to avoid wild extrapolation (s)

    private static final double kLatency = 0.02; // 20ms

    private Translation2d prevVelocity = new Translation2d();
    private Translation2d chassisAccel = new Translation2d();
    private double        prevOmega    = 0;
    private double        angularAccel = 0;

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    public Superstructure(SwerveSubsystem swerve,
                          FlywheelSubsystem leftFlywheel, FlywheelSubsystem rightFlywheel,
                          HoodSubsystem leftHood, HoodSubsystem rightHood,
                          IndexerSubsystem indexer,
                          IntakePivotSubsystem pivot, IntakeRollerSubsystem intake,
                          TurretSubsystem leftTurret, TurretSubsystem rightTurret) {
        this.swerve = swerve;
        this.leftFlywheel = leftFlywheel;
        this.rightFlywheel = rightFlywheel;
        this.leftHood = leftHood;
        this.rightHood = rightHood;
        this.leftTurret = leftTurret;
        this.rightTurret = rightTurret;
        this.indexer = indexer;
        this.pivot = pivot;
        this.intake = intake;
    }

    // =========================================================================
    // Public commands
    // =========================================================================

    public Command resetYaw() {
        return new InstantCommand(() -> swerve.resetHeadingButtonPressed());
    }

    public Command runIntake(boolean deployed) {
        return Commands.either(
            pivot.deploy().alongWith(intake.setVelocity(RotationsPerSecond.of(100))),
            pivot.retract().alongWith(intake.setVelocity(RotationsPerSecond.of(75))),
            () -> deployed
        );
    }

    public Command index(boolean withUnjam) {
        return Commands.either(
            indexer.setVelocity(() -> RotationsPerSecond.of(100))
                .withTimeout(1.5)
                .andThen(indexer.setVelocity(() -> RotationsPerSecond.of(-30)).withTimeout(0.125))
                .repeatedly(),
            indexer.setVelocity(() -> RotationsPerSecond.of(100)),
            () -> withUnjam
        );
    }

    public Command index() {
        return indexer.setVelocity(() -> RotationsPerSecond.of(100));
    }

    public Command collapseIntakeForScore() {
        return pivot.setManual(0.05).alongWith(intake.setPercentOutput(1.0));
    }

    public Command reverseFeeder() {
        return indexer.setPercentOutput(-1);
    }

    public Command setFlywheels(DoubleSupplier left, DoubleSupplier right) {
        return leftFlywheel.setVelocity(left).alongWith(rightFlywheel.setVelocity(right));
    }

    public Command setFlywheelsToTarget() {
        return leftFlywheel.setVelocity(() -> targetLeftFlywheel)
            .alongWith(rightFlywheel.setVelocity(() -> targetRightFlywheel));
    }

    public Command setHoods(DoubleSupplier left, DoubleSupplier right) {
        return leftHood.setRotations(left).alongWith(rightHood.setRotations(right));
    }

    public Command setHoodsToTarget() {
        return leftHood.setRotations(() -> targetLeftHood)
            .alongWith(rightHood.setRotations(() -> targetRightHood));
    }

    public Command stowHoods() {
        return leftHood.stow().alongWith(rightHood.stow());
    }

    public Command stowHoodsOnce() {
        return leftHood.stowOnce().alongWith(rightHood.stowOnce());
    }

    public Command setTurrets(DoubleSupplier left, DoubleSupplier right) {
        return leftTurret.setDegrees(left).alongWith(rightTurret.setDegrees(right));
    }

    public Command warmup() {
        return leftFlywheel.warmup().alongWith(rightFlywheel.warmup());
    }

    public Command track(Supplier<Translation2d> target) {
        return leftTurret.track(target).alongWith(rightTurret.track(target));
    }

    public Command track(Supplier<Translation2d> left, Supplier<Translation2d> right) {
        return leftTurret.track(left).alongWith(rightTurret.track(right));
    }

    public Command trackHub() {
        return leftTurret.trackHub().alongWith(rightTurret.trackHub());
    }

    public Command trackAppliedTarget() {
        return leftTurret.track(appliedLeftTargetSupplier)
            .alongWith(rightTurret.track(appliedRightTargetSupplier));
    }

    public Command stopShooters() {
        return leftFlywheel.stop().alongWith(
            rightFlywheel.stop(), leftHood.stow(), rightHood.stow(), indexer.stop());
    }

    // -------------------------------------------------------------------------
    // Readiness checks
    // -------------------------------------------------------------------------

    public boolean leftShooterReady() {
        return leftHood.atTarget() && leftFlywheel.atTargetVelocity() && leftTurret.atTarget();
    }

    public boolean rightShooterReady() {
        return rightHood.atTarget() && rightFlywheel.atTargetVelocity() && rightTurret.atTarget();
    }

    public boolean turretsCanShoot() {
        return leftTurret.atTarget(3) && rightTurret.atTarget(3);
    }

    public boolean shootersReady() {
        return leftShooterReady() && rightShooterReady();
    }

    public boolean hoodAndFlywheelsReady() {
        return leftHood.atTarget() && leftFlywheel.atTargetVelocity()
            && rightHood.atTarget() && rightFlywheel.atTargetVelocity();
    }

    // =========================================================================
    // Shooting commands
    // =========================================================================

    public Command shoot(Supplier<Translation2d> target) {
        return new DeferredCommand(() -> {
            this.targetSupplier = target;
            this.adjustTargetForMovingShots = true;
            return new ParallelCommandGroup(
                setFlywheels(() -> targetLeftFlywheel, () -> targetRightFlywheel),
                setHoods(() -> targetLeftHood, () -> targetRightHood),
                trackAppliedTarget(),
                (Commands.waitUntil(this::shootersReady)
                    .andThen(index(true)
                        .until(() -> !turretsCanShoot())
                        .alongWith(runIntake(false))
                    )).repeatedly()
            );
        }, Set.of(leftFlywheel, rightFlywheel, leftHood, rightHood, leftTurret, rightTurret, indexer, pivot, intake));
    }

    public Command shootStationary() {
        return new DeferredCommand(() -> {
            this.targetSupplier = kHubSupplier;
            this.adjustTargetForMovingShots = false;
            return new ParallelCommandGroup(
                setFlywheelsToTarget(),
                setHoodsToTarget(),
                trackHub(),
                (Commands.waitUntil(this::shootersReady)
                    .andThen(index(true)
                        .until(() -> !turretsCanShoot())
                        .alongWith(runIntake(false))
                    )).repeatedly()
            );
        }, Set.of(leftFlywheel, rightFlywheel, leftHood, rightHood, leftTurret, rightTurret, indexer, pivot, intake));
    }

    public Command shoot() {
        return this.shoot(targetSupplier);
    }

    public Command feedthrough(Supplier<Translation2d> target) {
        return new DeferredCommand(() -> {
            this.targetSupplier = target;
            this.adjustTargetForMovingShots = true;
            return new ParallelCommandGroup(
                setFlywheels(() -> targetLeftFlywheel, () -> targetRightFlywheel),
                setHoods(() -> targetLeftHood, () -> targetRightHood),
                trackAppliedTarget(),
                runIntake(true),
                (Commands.waitUntil(this::shootersReady).andThen(index(true))).repeatedly()
            );
        }, Set.of(leftFlywheel, rightFlywheel, leftHood, rightHood, leftTurret, rightTurret, indexer, pivot, intake));
    }

    public Command feedthrough() {
        return this.feedthrough(() -> FieldPositions.kHub.get());
    }

    public Command passFeedthrough() {
        return this.feedthrough(kPassSupplier);
    }

    public Command autoShoot() {
        return this.shoot().withTimeout(4.0);
    }

    public Command pass() {
        return this.forceShoot(kPassSupplier, 0.25);
    }

    public Command forceShoot(Supplier<Translation2d> target, double indexDelay) {
        return new DeferredCommand(() -> {
            this.targetSupplier = target;
            this.adjustTargetForMovingShots = false;
            return new ParallelCommandGroup(
                setFlywheels(() -> targetLeftFlywheel, () -> targetRightFlywheel),
                setHoods(() -> targetLeftHood, () -> targetRightHood),
                track(targetSupplier),
                new WaitCommand(indexDelay).andThen(index(true).alongWith(runIntake(false)))
            );
        }, Set.of(leftFlywheel, rightFlywheel, leftHood, rightHood, leftTurret, rightTurret, indexer, pivot, intake));
    }

    public Command forceShoot(double indexDelay) {
        return this.forceShoot(() -> FieldPositions.kHub.get(), indexDelay);
    }

    public Command manualShot(DoubleSupplier distance) {
        return new DeferredCommand(() -> {
            double[] parameters = shotTable.getShotParameters(distance.getAsDouble());
            return new ParallelCommandGroup(
                setFlywheels(() -> parameters[1], () -> parameters[1]),
                setHoods(() -> parameters[0], () -> parameters[0]),
                setTurrets(() -> 0, () -> 0),
                Commands.waitSeconds(1.0).andThen(index(true).alongWith(runIntake(false)))
            );
        }, Set.of(leftFlywheel, rightFlywheel, leftHood, rightHood, leftTurret, rightTurret, indexer, pivot, intake));
    }

    public Command shoot(double flywheel, double hood, boolean zeroTurret) {
        return new DeferredCommand(() -> new ParallelCommandGroup(
            setFlywheels(() -> flywheel, () -> flywheel),
            setHoods(() -> hood, () -> hood),
            Commands.either(setTurrets(() -> 0, () -> 0), setTurrets(() -> 180, () -> 180), () -> zeroTurret),
            Commands.waitUntil(this::shootersReady).andThen(index(true).alongWith(runIntake(false)))
        ), Set.of());
    }

    public Command shoot(DoubleSupplier distance, boolean zeroTurret) {
        return new DeferredCommand(() -> {
            double[] parameters = shotTable.getShotParameters(distance.getAsDouble());
            return new ParallelCommandGroup(
                setFlywheels(() -> parameters[1], () -> parameters[1]),
                setHoods(() -> parameters[0], () -> parameters[0]),
                Commands.either(setTurrets(() -> 0, () -> 0), setTurrets(() -> 180, () -> 180), () -> zeroTurret),
                Commands.waitUntil(this::hoodAndFlywheelsReady).andThen(index(true).alongWith(runIntake(false)))
            );
        }, Set.of(leftFlywheel, rightFlywheel, leftHood, rightHood, leftTurret, rightTurret, indexer, pivot, intake));
    }

    public Command shoot(double distance, boolean zeroTurret) {
        return this.shoot(() -> distance, zeroTurret);
    }

    // =========================================================================
    // periodic() — all prediction math lives here
    // =========================================================================

    @Override
    public void periodic() {

        // ---------------------------------------------------------------------
        // 1. Snapshot current chassis state
        // ---------------------------------------------------------------------

        Pose2d robotPose = swerve.getCurrentPose();
        Rotation2d robotRot = robotPose.getRotation();
        Translation2d robotXY = robotPose.getTranslation();

        ChassisSpeeds fieldSpeeds = swerve.getFieldRelativeChassisSpeeds();
        Translation2d currentVel = new Translation2d(fieldSpeeds.vxMetersPerSecond,
                                                       fieldSpeeds.vyMetersPerSecond);

        double currentOmega = fieldSpeeds.omegaRadiansPerSecond;

        // ---------------------------------------------------------------------
        // 2. Estimate linear and angular acceleration via EMA-filtered
        //    finite differences.
        //
        //    Raw accel = Δv / Δt.
        //    Filter with alpha = 0.3 to suppress odometry quantisation noise.
        //    The one-loop startup spike is absorbed by the filter in ~10 loops
        //    (~200 ms) — well before the first shot of a match.
        // ---------------------------------------------------------------------

        Translation2d rawAccel = currentVel.minus(prevVelocity).div(kDt);
        chassisAccel = new Translation2d(
            kAccelAlpha * rawAccel.getX() + (1.0 - kAccelAlpha) * chassisAccel.getX(),
            kAccelAlpha * rawAccel.getY() + (1.0 - kAccelAlpha) * chassisAccel.getY()
        );

        double rawAlphaDot = (currentOmega - prevOmega) / kDt;
        angularAccel = kAccelAlpha * rawAlphaDot + (1.0 - kAccelAlpha) * angularAccel;

        prevVelocity = currentVel;
        prevOmega = currentOmega;

        // ---------------------------------------------------------------------
        // 3. Current turret pivot positions in field frame
        // ---------------------------------------------------------------------

        Translation2d leftTurretPos  = robotXY.plus(kRobotToLeftTurret.toTranslation2d().rotateBy(robotRot));
        Translation2d rightTurretPos = robotXY.plus(kRobotToRightTurret.toTranslation2d().rotateBy(robotRot));

        // ---------------------------------------------------------------------
        // 4. Compute compensated aim points per turret
        // ---------------------------------------------------------------------

        Translation2d rawTarget = targetSupplier.get();

        if (adjustTargetForMovingShots) {
            Translation2d[] aimPoints = computePerTurretAimPoints(
                robotXY, robotRot,
                currentVel, chassisAccel,
                currentOmega, angularAccel,
                leftTurretPos, rightTurretPos,
                rawTarget
            );

            compensatedLeftTarget = aimPoints[0];
            compensatedRightTarget = aimPoints[1];
        } else {
            compensatedLeftTarget = rawTarget;
            compensatedRightTarget = rawTarget;
        }

        // ---------------------------------------------------------------------
        // 5. Look up shot parameters from each turret's predicted position
        // ---------------------------------------------------------------------

        double[] leftParams  = shotTable.getShotParameters(
            compensatedLeftTarget.minus(leftTurretPos).getNorm());
        double[] rightParams = shotTable.getShotParameters(
            compensatedRightTarget.minus(rightTurretPos).getNorm());

        targetLeftHood = leftParams[0];
        targetRightHood = rightParams[0];
        targetLeftFlywheel = leftParams[1];
        targetRightFlywheel = rightParams[1];

        // ---------------------------------------------------------------------
        // 6. Telemetry
        // ---------------------------------------------------------------------

        if (Robot.isSimulation()) {
            SmartDashboard.putNumber("SS/Accel X m/s2", chassisAccel.getX());
            SmartDashboard.putNumber("SS/Accel Y m/s2", chassisAccel.getY());
            SmartDashboard.putNumber("SS/Angular Accel rad/s2", angularAccel);

            Logger.recordOutput("SmartLogs/LeftCompensatedTarget",
                new Pose2d(compensatedLeftTarget, new Rotation2d()));
            Logger.recordOutput("SmartLogs/RightCompensatedTarget",
                new Pose2d(compensatedRightTarget, new Rotation2d()));
        }
    }

    // =========================================================================
    // computePerTurretAimPoints
    //
    // Dispatches to computeAimPoint() for each turret independently so that
    // their different field positions, and therefore different TOFs and rotation
    // offsets, are all accounted for separately.
    //
    // Returns Translation2d[] { leftAimPoint, rightAimPoint }
    // =========================================================================

    private Translation2d[] computePerTurretAimPoints(
            Translation2d robotXY, Rotation2d robotRot,
            Translation2d vel, Translation2d accel,
            double omega, double alpha,
            Translation2d leftTurretPos, Translation2d rightTurretPos,
            Translation2d target) {

        Translation2d leftAim = computeAimPoint(
            robotXY, robotRot, vel, accel, omega, alpha,
            kRobotToLeftTurret.toTranslation2d(), leftTurretPos, target);

        Translation2d rightAim = computeAimPoint(
            robotXY, robotRot, vel, accel, omega, alpha,
            kRobotToRightTurret.toTranslation2d(), rightTurretPos, target);

        return new Translation2d[]{ leftAim, rightAim };
    }

    // =========================================================================
    // computeAimPoint — single-turret closed-form solver with accel + rotation
    //
    // The core question: if the projectile leaves the turret now and takes
    // time T to reach the hub, where should the turret be pointing?
    //
    // The turret pivot will be at a different field position when the projectile
    // arrives because:
    //   (a) the robot centre translates by v*T + ½*a*T²
    //   (b) the robot rotates by ω*T + ½*α*T², shifting the turret offset vector
    //
    // We solve this iteratively:
    //
    //   Round 1 — first-guess T from current turret-to-target distance.
    //             Predict turret position at T using kinematics (a + b above).
    //
    //   Round 2 — recompute T from the predicted turret position (better distance).
    //             Predict turret position at the new T.
    //
    //   Result — the turret will have moved by Δp = predicted - current.
    //            Aim at (target - Δp) so the projectile, carried by Δp, lands
    //            on target.
    //
    // Two rounds converge to < 1 mm residual error at any realistic FRC speed
    // because T changes by < 2% between rounds.
    //
    // Parameters:
    //   robotXY               current robot centre (field frame)
    //   robotRot              current heading
    //   vel                   field-relative linear velocity (m/s)
    //   accel                 filtered linear acceleration (m/s²)
    //   omega                 angular velocity (rad/s)
    //   alpha                 filtered angular acceleration (rad/s²)
    //   turretOffsetRobotFrame robot-frame offset of this turret's pivot
    //   currentTurretPos      current field position of this turret's pivot
    //   target                raw target position (field frame)
    // =========================================================================

    private Translation2d computeAimPoint(
            Translation2d robotXY, Rotation2d robotRot,
            Translation2d vel, Translation2d accel,
            double omega, double alpha,
            Translation2d turretOffsetRobotFrame,
            Translation2d currentTurretPos,
            Translation2d target) {

        // Round 1 — first-guess TOF from current geometry
        double dist0 = target.minus(currentTurretPos).getNorm();
        double tof1  = Math.min(shotTableTof(dist0) + kLatency, kMaxPredictTof);

        Translation2d predictedRobot1  = predictPosition(robotXY, vel, accel, tof1);
        Rotation2d    predictedHead1   = predictHeading(robotRot, omega, alpha, tof1);
        Translation2d predictedTurret1 = predictedRobot1.plus(
            turretOffsetRobotFrame.rotateBy(predictedHead1));

        // Round 2 — refine TOF from the predicted turret position
        double dist1 = target.minus(predictedTurret1).getNorm();
        double tof2  = Math.min(shotTableTof(dist1) + kLatency, kMaxPredictTof);

        Translation2d predictedRobot2  = predictPosition(robotXY, vel, accel, tof2);
        Rotation2d    predictedHead2   = predictHeading(robotRot, omega, alpha, tof2);
        Translation2d predictedTurret2 = predictedRobot2.plus(
            turretOffsetRobotFrame.rotateBy(predictedHead2));

        // Net displacement of the turret pivot over the flight time
        Translation2d turretDisplacement = predictedTurret2.minus(currentTurretPos);

        // Aim point: the turret locks onto (target - displacement) so that
        // the projectile, which travels with the turret, lands on target.
        return target.minus(turretDisplacement);
    }

    // =========================================================================
    // Kinematic helpers
    // =========================================================================

    /**
     * Predict field-frame position at time t using constant-acceleration
     * kinematics: p(t) = p0 + v*t + 0.5*a*t^2
     */
    private static Translation2d predictPosition(
            Translation2d p0, Translation2d vel, Translation2d accel, double t) {
        return new Translation2d(
            p0.getX() + vel.getX() * t + 0.5 * accel.getX() * t * t,
            p0.getY() + vel.getY() * t + 0.5 * accel.getY() * t * t
        );
    }

    /**
     * Predict robot heading at time t using constant-angular-acceleration
     * kinematics: theta(t) = theta0 + omega*t + 0.5*alpha*t^2
     */
    private static Rotation2d predictHeading(
            Rotation2d theta0, double omega, double alpha, double t) {
        return theta0.plus(new Rotation2d(omega * t + 0.5 * alpha * t * t));
    }

    /**
     * Convenience wrapper: extract time-of-flight from the shot table.
     * Keeps getShotParameters()[2] calls out of the solver so the index
     * is only defined in one place.
     */
    private double shotTableTof(double dist) {
        return shotTable.getShotParameters(dist)[2];
    }
}