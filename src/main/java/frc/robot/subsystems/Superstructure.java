package frc.robot.subsystems;


import static edu.wpi.first.units.Units.*;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.frc1731.field.FieldPositions;
import frc.robot.Robot;
import frc.robot.commands.JiggleToPosition;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.indexer.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.shooter.flywheel.*;
import frc.robot.subsystems.shooter.hood.*;

public class Superstructure extends SubsystemBase {
    private SwerveSubsystem swerve;
    private FlywheelSubsystem flywheel;
    private HoodSubsystem hood;
    private IndexerSubsystem indexer;
    private KickerSubsystem kicker;
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

    private Translation2d compensatedTarget  = new Translation2d();

    // final lambdas — never reassigned, always read the latest field value
    public final Supplier<Translation2d> appliedTargetSupplier  = () -> compensatedTarget;

    // -------------------------------------------------------------------------
    // Shot parameter targets (written by periodic, read by commands via lambdas)
    // -------------------------------------------------------------------------

    private double targetHood = 0;
    private double targetFlywheel = 0;
    private boolean adjustTargetForMovingShots = false;
    private boolean trackTarget = false;

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
                          FlywheelSubsystem flywheel,
                          HoodSubsystem hood,
                          IndexerSubsystem indexer,
                          KickerSubsystem kicker,
                          IntakePivotSubsystem pivot, 
                          IntakeRollerSubsystem intake
                          ) {
        this.swerve = swerve;
        this.flywheel = flywheel;
        this.hood = hood;
        this.indexer = indexer;
        this.kicker = kicker;
        this.pivot = pivot;
        this.intake = intake;
    }

    // =========================================================================
    // Public commands
    // =========================================================================

    public Command resetSwerve() {
        return new InstantCommand(() -> {
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

    public Command stopShooters() {
        return flywheel.stop().alongWith(hood.stow(), indexer.stop(), kicker.stop());
    }

    public Command spit() {
        return pivot.deploy().alongWith(intake.setVelocity(RotationsPerSecond.of(-125)), indexer.eject(), kicker.eject());
    }

    // -------------------------------------------------------------------------
    // Readiness checks
    // -------------------------------------------------------------------------

    public boolean readyToShoot() {
        return hood.atTarget() && flywheel.atTargetVelocity();
    }

    // =========================================================================
    // Shooting commands
    // =========================================================================

    private Command shoot(Supplier<Translation2d> target, BooleanSupplier adjustForMovingShot, BooleanSupplier trackTarget, BooleanSupplier shotCondition) {
        return new InstantCommand(() -> {
            this.targetSupplier = target;
            this.adjustTargetForMovingShots = adjustForMovingShot.getAsBoolean();
            this.trackTarget = trackTarget.getAsBoolean();
        }).andThen(
            new ParallelCommandGroup(
                flywheel.setVelocity(() -> targetFlywheel),
                hood.setRotations(() -> targetHood),
                Commands.waitUntil(shotCondition)
            )
        );
    }

    // public Command shoot(Supplier<Translation2d> target) {
    //     return new DeferredCommand(() -> {
    //         this.targetSupplier = target;
    //         this.adjustTargetForMovingShots = true;
    //         return new ParallelCommandGroup(
    //             setFlywheels(() -> targetFlywheel, () -> targetRightFlywheel),
    //             setHoods(() -> targetHood, () -> targetRightHood),
    //             trackAppliedTarget(),
    //             intake.setVelocity(RotationsPerSecond.of(100)),
    //             (Commands.waitUntil(this::shootersReady)
    //                 .andThen(
    //                     index().until(() -> !turretsCanShoot())
    //                     .alongWith(new JiggleToPosition(pivot))
    //                 )).repeatedly()
    //         );
    //     }, Set.of(flywheel, hood, leftHood, rightHood, leftTurret, rightTurret, indexer, pivot, intake));
    // }

    // public Command stationaryShot() {
    //     return new DeferredCommand(() -> {
    //         this.targetSupplier = kHubSupplier;
    //         this.adjustTargetForMovingShots = false;
    //         return new ParallelCommandGroup(
    //             setFlywheelToTarget(),
    //             setHoodsToTarget(),
    //             trackHub(),
    //             intake.setVelocity(RotationsPerSecond.of(100)),
    //             (Commands.waitUntil(this::shootersReady)
    //                 .andThen(
    //                     index().until(() -> !turretsCanShoot())
    //                     .alongWith(new JiggleToPosition(pivot))
    //                 )).repeatedly()
    //         );
    //     }, Set.of(leftFlywheel, rightFlywheel, leftHood, rightHood, leftTurret, rightTurret, indexer, pivot, intake));
    // }

    // public Command shoot() {
    //     return this.shoot(targetSupplier);
    // }

    // public Command feedthrough(Supplier<Translation2d> target) {
    //     return new DeferredCommand(() -> {
    //         this.targetSupplier = target;
    //         this.adjustTargetForMovingShots = true;
    //         return new ParallelCommandGroup(
    //             setFlywheels(() -> targetFlywheel, () -> targetRightFlywheel),
    //             setHoods(() -> targetHood, () -> targetRightHood),
    //             trackAppliedTarget(),
    //             runIntake(true),
    //             (Commands.waitUntil(this::shootersReady).andThen(index())).repeatedly()
    //         );
    //     }, Set.of(leftFlywheel, rightFlywheel, leftHood, rightHood, leftTurret, rightTurret, indexer, pivot, intake));
    // }

    // public Command feedthrough() {
    //     return this.feedthrough(() -> FieldPositions.kHub.get());
    // }

    // public Command passFeedthrough() {
    //     return this.feedthrough(kPassSupplier);
    // }

    // public Command autoShoot() {
    //     return this.shoot().withTimeout(4.0);
    // }

    // public Command pass() {
    //     return this.shoot(kPassSupplier);
    // }

    // public Command forceShoot(Supplier<Translation2d> target, double indexDelay) {
    //     return new DeferredCommand(() -> {
    //         this.targetSupplier = target;
    //         this.adjustTargetForMovingShots = false;
    //         return new ParallelCommandGroup(
    //             setFlywheels(() -> targetFlywheel, () -> targetRightFlywheel),
    //             setHoods(() -> targetHood, () -> targetRightHood),
    //             track(targetSupplier),
    //             new WaitCommand(indexDelay).andThen(index().alongWith(runIntake(false)))
    //         );
    //     }, Set.of(leftFlywheel, rightFlywheel, leftHood, rightHood, leftTurret, rightTurret, indexer, pivot, intake));
    // }

    // public Command forceShoot(double indexDelay) {
    //     return this.forceShoot(() -> FieldPositions.kHub.get(), indexDelay);
    // }

    // public Command tuneShot(double flywheel, double hood, boolean zeroTurret) {
    //     return new DeferredCommand(() -> new ParallelCommandGroup(
    //         setFlywheels(() -> flywheel, () -> flywheel),
    //         setHoods(() -> hood, () -> hood),
    //         Commands.either(setTurrets(() -> 0, () -> 0), setTurrets(() -> 180, () -> 180), () -> zeroTurret),
    //         Commands.waitUntil(this::hoodAndFlywheelsReady).andThen(index().alongWith(runIntake(false)))
    //     ), Set.of());
    // }

    // public Command tuneShot(DoubleSupplier distance, boolean zeroTurret) {
    //     return new DeferredCommand(() -> {
    //         double[] parameters = shotTable.getShotParameters(distance.getAsDouble());
    //         return new ParallelCommandGroup(
    //             setFlywheels(() -> parameters[1], () -> parameters[1]),
    //             setHoods(() -> parameters[0], () -> parameters[0]),
    //             Commands.either(setTurrets(() -> 0, () -> 0), setTurrets(() -> 180, () -> 180), () -> zeroTurret),
    //             Commands.waitSeconds(1.0).andThen(index().alongWith(
    //                 new JiggleToPosition(pivot),
    //                 intake.setVelocity(RotationsPerSecond.of(100))
    //             ))
    //         );
    //     }, Set.of(leftFlywheel, rightFlywheel, leftHood, rightHood, leftTurret, rightTurret, indexer, pivot, intake));
    // }

    // public Command manualShot(double distance, boolean zeroTurret) {
    //     return this.tuneShot(() -> distance, zeroTurret);
    // }

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

        Translation2d rawTarget = targetSupplier.get();

        if (adjustTargetForMovingShots) {
            double tof1 = shotTableTof(rawTarget.minus(robotXY).getNorm());
            Translation2d firstPassTarget = rawTarget.minus(currentVel.times(tof1));
            double tof2 = shotTableTof(firstPassTarget.minus(robotXY).getNorm());
            Translation2d secondPassTarget = rawTarget.minus(currentVel.times(tof2));

            compensatedTarget = secondPassTarget;
        } else {
            compensatedTarget = rawTarget;
        }

        // ---------------------------------------------------------------------
        // 5. Look up shot parameters from each turret's predicted position
        // ---------------------------------------------------------------------

        double[] shotParams  = shotTable.getShotParameters(compensatedTarget.minus(robotXY).getNorm());

        targetHood = shotParams[0];
        targetFlywheel = shotParams[1];
    }

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