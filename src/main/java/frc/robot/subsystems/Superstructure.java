package frc.robot.subsystems;


import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.lib.frc1731.field.FieldPositions;
import frc.robot.Robot;
import frc.robot.commands.JiggleToPosition;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.indexer.*;
import frc.robot.subsystems.intake.pivot.IntakePivotSubsystem;
import frc.robot.subsystems.intake.roller.IntakeRollerSubsystem;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.shooter.flywheel.*;
import frc.robot.subsystems.shooter.hood.*;
import frc.robot.subsystems.squeezer.SqueezerSubsystem;

public class Superstructure extends SubsystemBase {
    private SwerveSubsystem swerve;
    private FlywheelSubsystem flywheel;
    private HoodSubsystem hood;
    private IndexerSubsystem indexer;
    private KickerSubsystem kicker;
    private IntakePivotSubsystem pivot;
    private IntakeRollerSubsystem intake;
    private SqueezerSubsystem squeezer;

    private final ShotTable shotTable = ShotTable.getScoringTable();

    public final Supplier<Translation2d> kHubSupplier = () -> Robot.isRedAlliance()
        ? new Translation2d(11.91, 4.03)
        : new Translation2d(4.62, 4.03);

    public final Supplier<Translation2d> kPassSupplier = () -> {
        double x = Robot.isRedAlliance() ? FieldPositions.kFieldLength - 2 : 2;
        double y = Robot.isRedAlliance() && swerve.getCurrentPose().getY() > FieldPositions.kFieldWidth / 2.0 ? FieldPositions.kFieldWidth - 2 : 2;
        return new Translation2d(x, y);
    };

    private Supplier<Translation2d> targetSupplier = kHubSupplier;

    private Translation2d compensatedTarget  = new Translation2d();

    public final Supplier<Translation2d> appliedTargetSupplier  = () -> compensatedTarget;

    private double targetHood = 0;
    private double targetFlywheel = 0;
    private boolean adjustTargetForMovingShots = false;

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    public Superstructure(SwerveSubsystem swerve,
                          FlywheelSubsystem flywheel,
                          HoodSubsystem hood,
                          IndexerSubsystem indexer,
                          KickerSubsystem kicker,
                          IntakePivotSubsystem pivot, 
                          IntakeRollerSubsystem intake,
                          SqueezerSubsystem squeezer
                          ) {
        this.swerve = swerve;
        this.flywheel = flywheel;
        this.hood = hood;
        this.indexer = indexer;
        this.kicker = kicker;
        this.pivot = pivot;
        this.intake = intake;
        this.squeezer = squeezer;
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
            pivot.deploy().alongWith(intake.setPercentOutput(1.0)),
            pivot.retract().alongWith(intake.setPercentOutput(1.0)),
            () -> deployed
        );
    }

    public Command stopShooters() {
        return flywheel.stop().alongWith(hood.stow(), indexer.stop(), kicker.stop(), swerve.stopLocking());
    }

    public Command spit() {
        return pivot.deploy().alongWith(intake.setPercentOutput(-1.0), indexer.eject(), kicker.eject());
    }

    public Command lockSwerveToHub() {
        return swerve.lockHeadingTarget(appliedTargetSupplier);
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

    private Command shoot(Supplier<Translation2d> target, BooleanSupplier adjustForMovingShot, BooleanSupplier trackTarget, BooleanSupplier feedthrough, BooleanSupplier shotCondition, BooleanSupplier squeeze) {
        return new InstantCommand(() -> {
            this.targetSupplier = target;
            this.adjustTargetForMovingShots = adjustForMovingShot.getAsBoolean();
        }).andThen(
            swerve.setHeadingTarget(() -> compensatedTarget),
            swerve.setLockingEnabled(trackTarget.getAsBoolean()),
            new ParallelCommandGroup(
                flywheel.setVelocity(() -> targetFlywheel),
                hood.setRotations(() -> targetHood),
                Commands.waitUntil(shotCondition).andThen(
                    new ParallelCommandGroup( // Only start the feeding sequence after we are ready to shoot
                        indexer.feed(),
                        kicker.setVelocity(() -> targetFlywheel),
                        Commands.either(squeezer.squeeze(), Commands.none(), squeeze)
                    )
                ),
                Commands.either( // If feedthrough continue intaking, otherwise jiggle
                    this.runIntake(true),
                    Commands.waitUntil(shotCondition)
                        .andThen(new JiggleToPosition(pivot).alongWith(intake.setPercentOutput(1.0))),
                    feedthrough
                )
            )
        );
    }

    private Command shoot(DoubleSupplier targetFlywheel, DoubleSupplier targetHood, BooleanSupplier shotCondition) {
        return new ParallelCommandGroup(
            flywheel.setVelocity(targetFlywheel.getAsDouble()),
            hood.setRotations(targetHood.getAsDouble()),
            Commands.waitUntil(shotCondition) .andThen(
                new JiggleToPosition(pivot).alongWith(
                    indexer.feed(),
                    intake.setPercentOutput(1.0),
                    kicker.setVelocity(targetFlywheel.getAsDouble()),
                    squeezer.squeeze()
                )
            )
            )
        ;
    }

    public Command autoShoot(boolean feedthrough) {
        return new InstantCommand(() -> {
            this.targetSupplier = kHubSupplier;
            this.adjustTargetForMovingShots = true;
        }).andThen(
            swerve.setHeadingTarget(() -> compensatedTarget),
            swerve.setLockingEnabled(true),
            new ParallelCommandGroup(
                flywheel.setVelocity(() -> targetFlywheel),
                swerve.lockHeadingTarget(() -> compensatedTarget).withInterruptBehavior(InterruptionBehavior.kCancelSelf),
                hood.setRotations(() -> targetHood),
                Commands.waitUntil(this::readyToShoot).andThen(
                    new ParallelCommandGroup( // Only start the feeding sequence after we are ready to shoot
                        indexer.feed(),
                        kicker.setVelocity(() -> targetFlywheel),
                        squeezer.squeeze()
                    )
                ),
                Commands.either( // If feedthrough continue intaking, otherwise jiggle
                    this.runIntake(true),
                    Commands.waitUntil(this::readyToShoot)
                        .andThen(new JiggleToPosition(pivot).alongWith(intake.setPercentOutput(1.0))),
                    () -> feedthrough
                )
            )
        );
    }

    public Command shoot() {
        return shoot(kHubSupplier, () -> true, () -> true, () -> false, this::readyToShoot, () -> true);
    }

    public Command stationaryShot() {
        return shoot(kHubSupplier, () -> false, () -> true, () -> false, this::readyToShoot, () -> true);
    }

    public Command feedthrough() {
        return shoot(kHubSupplier, () -> true, () -> true, () -> true, this::readyToShoot, () -> false);
    }

    public Command pass() {
        return shoot(kPassSupplier, () -> true, () -> true, () -> false, this::readyToShoot, () -> true);
    }

    public Command passFeedthrough() {
        return shoot(kPassSupplier, () -> true, () -> true, () -> true, this::readyToShoot, () -> false);
    }

    public Command defaultShot(DoubleSupplier flywheelRPS, DoubleSupplier hoodRotations) {
        return shoot(flywheelRPS, hoodRotations, this::readyToShoot);
    }

    public Command defaultShot(double flywheelRPS, double hoodRotations) {
        return shoot(() -> flywheelRPS, () -> hoodRotations, this::readyToShoot);
    }

    public Command defaultShot(double distance) {
        return defaultShot(() -> shotTable.getShotParameters(distance)[1], () -> shotTable.getShotParameters(distance)[0]);
    }

    public Command warmup() {
        return flywheel.setVelocity(() -> targetFlywheel).alongWith(hood.setRotations(() -> targetHood));
    }

    @Override
    public void periodic() {
        Pose2d robotPose = swerve.getCurrentPose();
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

        double distance = compensatedTarget.minus(robotXY).getNorm();

        double[] shotParams  = shotTable.getShotParameters(distance);

        if (Robot.isSimulation()) {
            Logger.recordOutput("TargetDistance", distance);
            Logger.recordOutput("CompensatedTarget", new Pose2d(compensatedTarget, Rotation2d.kZero));
        }

        this.targetHood = shotParams[0];
        this.targetFlywheel = shotParams[1];
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