package frc.robot.subsystems;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.frc1731.Utils;
import frc.lib.frc6328.FieldConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.shooter.ShotProfile;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;

public class Superstructure extends SubsystemBase {
    private SwerveSubsystem swerve;
    private FlywheelSubsystem flywheel;
    private HoodSubsystem hood;
    private TurretSubsystem turret;
    private IndexerSubsystem indexer;
    private IntakePivotSubsystem pivot;
    private IntakeRollerSubsystem intake;

    // private ShotTable shotTable = new ShotTable();

    public static ShotProfile hubShotProfile = new ShotProfile(0, 0, 0, false);

    private static Supplier<Pose2d> hubSupplier = () -> new Pose2d(Utils.flip(FieldConstants.Hub.topCenterPoint.toTranslation2d()), new Rotation2d());
    private static Supplier<Pose2d> passSupplier = () -> new Pose2d(Utils.flip(new Pose2d().getTranslation()), new Rotation2d());

    public Superstructure(SwerveSubsystem swerve, FlywheelSubsystem flywheel, HoodSubsystem hood, TurretSubsystem turret, 
                            IndexerSubsystem indexer, IntakePivotSubsystem pivot, IntakeRollerSubsystem intake) {
        this.swerve = swerve;
        this.flywheel = flywheel;
        this.hood = hood;
        this.turret = turret;
        this.indexer = indexer;
        this.pivot = pivot;
        this.intake = intake;
    }

    public Command intake() {
        return pivot.deploy()
        .alongWith(
            intake.setPercent(1)
        );
    }

    public Command intakeWithPopcorn() {
        return pivot.deploy()
        .alongWith(
            intake.setPercent(1),
            indexer.setPercent(0.25),
            turret.setDegrees(0, 0)
        );
    }

    public Command shoot() {
        return this.shoot(hubSupplier);
    }

    public Command shoot(Supplier<Pose2d> targetPose) {
        return new ParallelCommandGroup(
            flywheel.setVelocity(80, 80),
            pivot.retract(),
            intake.setPercent(0.75),
            hood.setRotations(2, 2),
            turret.setDegrees(() -> swerve.getYaw(), () -> swerve.getYaw()),
            Commands.defer(() -> {
                if (flywheel.atBothTargetVelocity() && hood.atBothTargetRotations() && turret.atBothTargetRotations()) {
                    return indexer.setPercent(1);
                } else {
                    return indexer.setPercent(0);
                }
            }, Set.of(indexer))
        );
    }

    public Command feedthrough() {
        return new ParallelCommandGroup(
            flywheel.setVelocity(80, 80),
            pivot.deploy(),
            intake.setPercent(1d),
            hood.setRotations(2, 2),
            turret.setDegrees(() -> swerve.getYaw(), () -> swerve.getYaw()),
            Commands.defer(() -> {
                if (flywheel.atBothTargetVelocity() && hood.atBothTargetRotations() && turret.atBothTargetRotations()) {
                    return indexer.setPercent(1);
                } else {
                    return indexer.setPercent(0);
                }
            }, Set.of(indexer))
        );
    }

    public Command pass() {
        return this.shoot(passSupplier);
    }

    public Command aimTurret() {
        return turret.aim(() -> swerve.getCurrentPose(), hubSupplier);
    }

    public Command warmup() {
        return flywheel.setVelocity(50, 50);
    }

    public Command unjam() {
        return Commands.none();
    }

    @Override
    public void periodic() {
        // Add any periodic code here
    }
}