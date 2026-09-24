package frc.robot.subsystems;


import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.frc1731.field.FieldPositions;
import frc.lib.frc6328.LoggedTunableNumber;
import frc.robot.Robot;
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
    private final ShotTable passTable = ShotTable.getPassingTable();

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

    private boolean isPassing = false;

    private LoggedTunableNumber tuneableFlywheelRPS = new LoggedTunableNumber("Tuned Flywheel RPS", 0.0, () -> true);
    private LoggedTunableNumber tuneableHoodRotations = new LoggedTunableNumber("Tuned Hood Rotations", 0.0, () -> true);

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

    public Command stageFuel() {
        return new ParallelCommandGroup(
            kicker.feed().withTimeout(0.25).andThen(kicker.stop()),
            indexer.feed().withTimeout(0.25).andThen(indexer.stop())
        );
    }

    public Command resetSwerve() {
        return new InstantCommand(() -> {
            swerve.resetTelePose();
        });
    }

    public Command intake() {
        return pivot.deploy()
        .alongWith(intake.intake())
        .alongWith(
            kicker.setPercent(0.05), 
            indexer.setPercent(0.05)
        );
    }

    public Command stopShooters() {
        return flywheel.stop().alongWith(hood.stow(), indexer.stop(), kicker.stop());
    }

    public Command spit() {
        return pivot.deploy().alongWith(intake.eject(), indexer.eject(), kicker.eject());
    }

    public Command lockSwerveToHub() {
        return swerve.lockHeadingTarget(appliedTargetSupplier);
    }

    // -------------------------------------------------------------------------
    // Readiness checks
    // -------------------------------------------------------------------------

    public boolean readyToShoot() {
      //  return true;
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
                        kicker.feed(),
                        Commands.either(Commands.waitSeconds(1.5).andThen(squeezer.squeeze()), Commands.none(), squeeze)
                    )
                ),
                Commands.either( // If feedthrough continue intaking, otherwise jiggle
                    pivot.deploy().alongWith(intake.intake()),
                    Commands.waitUntil(shotCondition)
                        .andThen(pivot.jiggle().alongWith(intake.intake())),
                    feedthrough
                )
            )
        );
    }

    private Command shoot(DoubleSupplier targetFlywheel, DoubleSupplier targetHood, BooleanSupplier shotCondition) {
        return new ParallelCommandGroup(
            flywheel.setVelocity(targetFlywheel.getAsDouble()),
            hood.setRotations(targetHood.getAsDouble()),
            Commands.waitUntil(shotCondition).andThen(
                pivot.jiggle().alongWith(
                    indexer.feed(),
                    intake.intake(),
                    kicker.feed(),
                    Commands.waitSeconds(1.5).andThen(squeezer.squeeze())
                )
            )
            )
        ;
    }

    public Command autoShoot() {
        return new DeferredCommand(() -> {
            this.adjustTargetForMovingShots = true;
            this.targetSupplier = kHubSupplier;
            return new ParallelCommandGroup(
                flywheel.setVelocity(()-> targetFlywheel),
                hood.setRotations(() -> targetHood),
                Commands.waitUntil(this::readyToShoot).andThen(
                    indexer.feed().alongWith(kicker.feed(), pivot.jiggle())
                )
            );
        }, 
        Set.of(flywheel, hood, indexer, kicker, pivot));
    }

    public Command tunedShot() {
        return new ParallelCommandGroup(
            flywheel.setVelocity(tuneableFlywheelRPS),
            hood.setRotations(tuneableHoodRotations),
            // Allow the live setpoints and periodic readiness telemetry to update first.
            Commands.waitSeconds(0.04)
                .andThen(Commands.waitUntil(() -> tuneableFlywheelRPS.get() > 0 && readyToShoot()))
                .andThen(
                indexer.feed().alongWith(
                    kicker.setTipSpeedMPS(flywheel::getTargetTipSpeedMPS),
                    pivot.jiggle()
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
        return new InstantCommand(() -> isPassing = true).andThen(
            shoot(kPassSupplier, () -> true, () -> true, () -> false, this::readyToShoot, () -> true)
        ).finallyDo(() -> isPassing = false);
    }

    public Command passFeedthrough() {
        return new InstantCommand(() -> isPassing = true).andThen(
            shoot(kPassSupplier, () -> true, () -> true, () -> true, this::readyToShoot, () -> true)
        ).finallyDo(() -> isPassing = false);
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
        ShotTable appliedTable = shotTable;
        if (isPassing) {
            appliedTable = passTable;
        }

        double[] shotParams  = appliedTable.getShotParameters(distance);

        SmartDashboard.putNumber("TargetDistance", distance);
        // if (Robot.isSimulation()) {
        //     Logger.recordOutput("TargetDistance", distance);
        //     Logger.recordOutput("CompensatedTarget", new Pose2d(compensatedTarget, Rotation2d.kZero));
        // }

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
