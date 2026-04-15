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
import frc.robot.commands.JiggleToPosition;
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
        ? new Translation2d(11.91, 4.03)
        : new Translation2d(4.62, 4.03);

    private final Supplier<Translation2d> kPassSupplier = () -> {
        double x = Robot.isRedAlliance() ? FieldPositions.kFieldLength - 2 : 2;
        double y = Robot.isRedAlliance() && swerve.getCurrentPose().getY() > FieldPositions.kFieldWidth / 2.0 ? FieldPositions.kFieldWidth - 2 : 2;
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

    // private static final double kMaxPredictTof = 2.2;   // clamp TOF to avoid wild extrapolation (s)
    // private static final double kLatency = 0.02; // 20ms
    // private static final double kCompGain = 1.0; // 0 = no compensation, 1 = full compensation

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

    public Command resetSwerve() {
        return new InstantCommand(() -> {
            // swerve.resetHeadingButtonPressed();
            swerve.resetTelePose();
        });
    }

    public Command runIntake(boolean deployed) {
        return Commands.either(
            pivot.deploy().alongWith(intake.setVelocity(RotationsPerSecond.of(125))),
            pivot.retract().alongWith(intake.setVelocity(RotationsPerSecond.of(100))),
            () -> deployed
        );
    }

    public Command index() {
        return indexer.index();
    }

    public Command unjamIndexer() {
        return index().withTimeout(0.5).andThen(index().withTimeout(0.25)).repeatedly();
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

    public Command  warmup() {
        return new ParallelCommandGroup(
            leftFlywheel.warmup(),
            rightFlywheel.warmup(),
            leftHood.stow(),
            rightHood.stow(),
            leftTurret.trackHub(),
            rightTurret.trackHub()
        );
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

    public Command spit() {
        return pivot.deploy().alongWith(intake.setVelocity(RotationsPerSecond.of(-125)), indexer.setPercent(-1, -1));
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
                intake.setVelocity(RotationsPerSecond.of(100)),
                (Commands.waitUntil(this::shootersReady)
                    .andThen(
                        index().until(() -> !turretsCanShoot())
                        .alongWith(new JiggleToPosition(pivot))
                    )).repeatedly()
            );
        }, Set.of(leftFlywheel, rightFlywheel, leftHood, rightHood, leftTurret, rightTurret, indexer, pivot, intake));
    }

    public Command stationaryShot() {
        return new DeferredCommand(() -> {
            this.targetSupplier = kHubSupplier;
            this.adjustTargetForMovingShots = false;
            return new ParallelCommandGroup(
                setFlywheelsToTarget(),
                setHoodsToTarget(),
                trackHub(),
                intake.setVelocity(RotationsPerSecond.of(100)),
                (Commands.waitUntil(this::shootersReady)
                    .andThen(
                        index().until(() -> !turretsCanShoot())
                        .alongWith(new JiggleToPosition(pivot))
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
                (Commands.waitUntil(this::shootersReady).andThen(index())).repeatedly()
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
        return this.shoot(kPassSupplier);
    }

    public Command forceShoot(Supplier<Translation2d> target, double indexDelay) {
        return new DeferredCommand(() -> {
            this.targetSupplier = target;
            this.adjustTargetForMovingShots = false;
            return new ParallelCommandGroup(
                setFlywheels(() -> targetLeftFlywheel, () -> targetRightFlywheel),
                setHoods(() -> targetLeftHood, () -> targetRightHood),
                track(targetSupplier),
                new WaitCommand(indexDelay).andThen(index().alongWith(runIntake(false)))
            );
        }, Set.of(leftFlywheel, rightFlywheel, leftHood, rightHood, leftTurret, rightTurret, indexer, pivot, intake));
    }

    public Command forceShoot(double indexDelay) {
        return this.forceShoot(() -> FieldPositions.kHub.get(), indexDelay);
    }

    public Command tuneShot(double flywheel, double hood, boolean zeroTurret) {
        return new DeferredCommand(() -> new ParallelCommandGroup(
            setFlywheels(() -> flywheel, () -> flywheel),
            setHoods(() -> hood, () -> hood),
            Commands.either(setTurrets(() -> 0, () -> 0), setTurrets(() -> 180, () -> 180), () -> zeroTurret),
            Commands.waitUntil(this::hoodAndFlywheelsReady).andThen(index().alongWith(runIntake(false)))
        ), Set.of());
    }

    public Command tuneShot(DoubleSupplier distance, boolean zeroTurret) {
        return new DeferredCommand(() -> {
            double[] parameters = shotTable.getShotParameters(distance.getAsDouble());
            return new ParallelCommandGroup(
                setFlywheels(() -> parameters[1], () -> parameters[1]),
                setHoods(() -> parameters[0], () -> parameters[0]),
                Commands.either(setTurrets(() -> 0, () -> 0), setTurrets(() -> 180, () -> 180), () -> zeroTurret),
                Commands.waitSeconds(1.0).andThen(index().alongWith(
                    new JiggleToPosition(pivot),
                    intake.setVelocity(RotationsPerSecond.of(100))
                ))
            );
        }, Set.of(leftFlywheel, rightFlywheel, leftHood, rightHood, leftTurret, rightTurret, indexer, pivot, intake));
    }

    public Command manualShot(double distance, boolean zeroTurret) {
        return this.tuneShot(() -> distance, zeroTurret);
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

        // double currentOmega = fieldSpeeds.omegaRadiansPerSecond;

        // ---------------------------------------------------------------------
        // 2. Estimate linear and angular acceleration via EMA-filtered
        //    finite differences.
        //
        //    Raw accel = Δv / Δt.
        //    Filter with alpha = 0.3 to suppress odometry quantisation noise.
        //    The one-loop startup spike is absorbed by the filter in ~10 loops
        //    (~200 ms) — well before the first shot of a match.
        // ---------------------------------------------------------------------

        // Translation2d rawAccel = currentVel.minus(prevVelocity).div(kDt);
        // chassisAccel = new Translation2d(
        //     kAccelAlpha * rawAccel.getX() + (1.0 - kAccelAlpha) * chassisAccel.getX(),
        //     kAccelAlpha * rawAccel.getY() + (1.0 - kAccelAlpha) * chassisAccel.getY()
        // );

        // double rawAlphaDot = (currentOmega - prevOmega) / kDt;
        // angularAccel = kAccelAlpha * rawAlphaDot + (1.0 - kAccelAlpha) * angularAccel;

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
            // Translation2d[] aimPoints = computePerTurretAimPoints(
            //     robotXY, robotRot,
            //     currentVel,
            //     currentOmega,
            //     leftTurretPos, rightTurretPos,
            //     rawTarget
            // );

            // compensatedLeftTarget = aimPoints[0];
            // compensatedRightTarget = aimPoints[1];
            double tof1Left = shotTableTof(rawTarget.minus(leftTurretPos).getNorm());
            Translation2d firstPassTargetLeft = rawTarget.minus(currentVel.times(tof1Left));
            double tof2Left = shotTableTof(firstPassTargetLeft.minus(leftTurretPos).getNorm());
            Translation2d secondPassTargetLeft = rawTarget.minus(currentVel.times(tof2Left));

            double tof1Right = shotTableTof(rawTarget.minus(rightTurretPos).getNorm());
            Translation2d firstPassTargetRight = rawTarget.minus(currentVel.times(tof1Right));
            double tof2Right = shotTableTof(firstPassTargetRight.minus(rightTurretPos).getNorm());
            Translation2d secondPassTargetRight = rawTarget.minus(currentVel.times(tof2Right));

            compensatedLeftTarget = secondPassTargetLeft;
            compensatedRightTarget = secondPassTargetRight;
        } else {
            compensatedLeftTarget = rawTarget;
            compensatedRightTarget = rawTarget;
        }

        Logger.recordOutput("Superstructure/LeftCompensatedTarget", new Pose2d(compensatedLeftTarget, new Rotation2d()));
        Logger.recordOutput("Superstructure/RightCompensatedTarget", new Pose2d(compensatedRightTarget, new Rotation2d()));

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
         //   Logger.recordOutput("SmartLogs/LeftCompensatedTarget",
        //        new Pose2d(compensatedLeftTarget, new Rotation2d()));
         //   Logger.recordOutput("SmartLogs/RightCompensatedTarget",
         //       new Pose2d(compensatedRightTarget, new Rotation2d()));
        }

        SmartDashboard.putNumber("Left Distance", rawTarget.getDistance(leftTurretPos));
        SmartDashboard.putNumber("Right Distance", rawTarget.getDistance(rightTurretPos));

        SmartDashboard.putNumber("Left Error", leftTurret.getError());
        SmartDashboard.putNumber("Right Error", rightTurret.getError());
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

    // private Translation2d[] computePerTurretAimPoints(
    //         Translation2d robotXY,
    //         Rotation2d robotRot,
    //         Translation2d vel,
    //         double omega,
    //         Translation2d leftTurretPos,
    //         Translation2d rightTurretPos,
    //         Translation2d target) {

    //     Translation2d leftAim = computeAimPoint(
    //         robotXY, robotRot, vel, omega,
    //         kRobotToLeftTurret.toTranslation2d(),
    //         leftTurretPos,
    //         target
    //     );

    //     Translation2d rightAim = computeAimPoint(
    //         robotXY, robotRot, vel, omega,
    //         kRobotToRightTurret.toTranslation2d(),
    //         rightTurretPos,
    //         target
    //     );

    //     return new Translation2d[]{ leftAim, rightAim };
    // }

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

    // private Translation2d computeAimPoint(
    //         Translation2d robotXY,
    //         Rotation2d robotRot,
    //         Translation2d vel,
    //         double omega,
    //         Translation2d turretOffsetRobotFrame,
    //         Translation2d currentTurretPos,
    //         Translation2d target) {

    //     // ---------------------------------------------------------------------
    //     // 1. Get distance + TOF (single pass, no iteration)
    //     // ---------------------------------------------------------------------
    //     double dist = target.minus(currentTurretPos).getNorm();
    //     double tof = Math.min(shotTableTof(dist) + kLatency, kMaxPredictTof);

    //     // ---------------------------------------------------------------------
    //     // 2. Predict robot future pose (NO ACCEL)
    //     // ---------------------------------------------------------------------
    //     Translation2d predictedRobot =
    //         robotXY.plus(vel.times(tof));

    //     Rotation2d predictedHeading =
    //         robotRot.plus(new Rotation2d(omega * tof));

    //     // ---------------------------------------------------------------------
    //     // 3. Predict turret future position
    //     // ---------------------------------------------------------------------
    //     Translation2d predictedTurret =
    //         predictedRobot.plus(turretOffsetRobotFrame.rotateBy(predictedHeading));

    //     // ---------------------------------------------------------------------
    //     // 4. Compute displacement and apply damping
    //     // ---------------------------------------------------------------------
    //     Translation2d turretDisplacement =
    //         predictedTurret.minus(currentTurretPos).times(kCompGain);

    //     // ---------------------------------------------------------------------
    //     // 5. Return compensated aim point
    //     // ---------------------------------------------------------------------
    //     return target.minus(turretDisplacement);
    // }

    // =========================================================================
    // Kinematic helpers
    // =========================================================================

    /**
     * Convenience wrapper: extract time-of-flight from the shot table.
     * Keeps getShotParameters()[2] calls out of the solver so the index
     * is only defined in one place.
     */
    private double shotTableTof(double dist) {
        return shotTable.getShotParameters(dist)[2];
    }
}