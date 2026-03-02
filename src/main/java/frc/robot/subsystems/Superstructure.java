package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.frc1731.Utils;
import frc.lib.frc6328.FieldConstants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.feeder.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.shooter.ShotProfile;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodConstants;
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

    public Command resetGyroCommand() {
        return new InstantCommand(() -> swerve.resetPose(
            Robot.isRedAlliance() ? new Pose2d(10.38, 3.01, new Rotation2d(Math.toRadians(0)))
                : new Pose2d(7.168, 5.006, new Rotation2d(Math.toRadians(180)))
        ));
    }

    public Command intakeCommand() {
        return pivot.deployCommand().alongWith(intake.setPercentOutputCommand(1));
    }

    public Command shootCommand() {
        return new ParallelCommandGroup(
            // flywheel.setFlywheelPercentCommand(1.0, 1.0),
            flywheel.setFlywheelVelocityCommand(80, 80),
            pivot.retractCommand(),
            intake.setPercentOutputCommand(0.75),
            hood.setHoodCommand(2, 2),
            turret.setRightTurretCommand(() -> swerve.getYaw()),
            // Commands.waitUntil(() -> flywheel.atRightTargetVelocity() && hood.atBothTarget())
            Commands.waitSeconds(2)
            // Commands.waitSeconds(2)
            .andThen(indexer.setPercentOutputCommand(1.0))
        );
    }

    // public Command passCommand() {
    //     return new ParallelCommandGroup(
    //         flywheel.setFlywheelPercentCommand(0.5, 0.5),
    //         pivot.retractCommand(),
    //         intake.setPercentOutputCommand(0.75),
    //         hood.setHoodCommand(25, 25),
    //         // Commands.waitUntil(() -> flywheel.atRightTargetVelocity() && hood.atRightTarget())
    //         Commands.waitSeconds(1.5)
    //         .andThen(indexer.setPercentOutputCommand(1.0))
    //     );
    // }

    // public Command shootCommand(Supplier<Pose2d> targetPose) {
    //     return new ParallelCommandGroup(
    //         flywheel.setFlywheelPercentCommand(1.0, 1.0),
    //         pivot.retractCommand(),
    //         intake.setPercentOutputCommand(0.5),
    //         hood.setHoodCommand(25, 25),
    //         // Commands.waitUntil(() -> flywheel.atRightTargetVelocity() && hood.atRightTarget())
    //         Commands.waitSeconds(1.5)
    //         .andThen(indexer.setPercentOutputCommand(1.0))
    //     );
    // }

    public Command feedthroughCommand() {
        return new ParallelCommandGroup(
            flywheel.setFlywheelVelocityCommand(50, 50),
            pivot.deployCommand(),
            intake.setPercentOutputCommand(1),
            hood.setHoodCommand(2, 2),
            Commands.waitUntil(() -> flywheel.atRightTargetVelocity() && hood.atRightTarget())
            .andThen(indexer.setPercentOutputCommand(1.0))
        );
    }

    public Command aimTurretCommand() {
        return turret.aimCommand(() -> swerve.getCurrentPose(), hubSupplier);
    }

    public Command unjamCommand() {
        return Commands.none();
    }

    // private double getTurretShotAngle() {
    //     Translation2d robotPose = swerve.getCurrentPose().getTranslation();
    //     Translation2d hubPose = Field.HUB.getPosition().toTranslation2d();
    //     double aimRadians = Math.atan((hubPose.getX() - robotPose.getX()) / (hubPose.getY() - robotPose.getY()));
    //     return Math.toDegrees(aimRadians) - swerve.getCurrentPose().getRotation().getDegrees();
    // }

    // private double getLeftShooterDistanceToHub() {
    //     Translation2d hub = FieldConstants.Hub.topCenterPoint.toTranslation2d();
    //     Pose3d leftShooter = turret.getLeftTurretPose(new Pose3d(swerve.getCurrentPose()));
    //     return hub.getDistance(leftShooter.getTranslation().toTranslation2d());
    // }

    // private double getRightShooterDistanceToHub() {
    //     Translation2d hub = FieldConstants.Hub.topCenterPoint.toTranslation2d();
    //     Pose3d rightShooter = turret.getRightTurretPose(new Pose3d(swerve.getCurrentPose()));
    //     return hub.getDistance(rightShooter.getTranslation().toTranslation2d());
    // }

    // public Command aimTurretAtHubCommand() {
    //     return turret.aimCommand(() -> swerve.getCurrentPose(), hubSupplier);
    // }

    // public Command aimTurretForPassCommand() {
    //     return turret.aimCommand(() -> swerve.getCurrentPose(), passSupplier);
    // }

    // public Command shootFuelCommand() {
    //     return new DeferredCommand(
    //         () -> {
    //             double[] leftParameters = shotTable.getShotParameters(getLeftShooterDistanceToHub());
    //             double[] rightParameters = shotTable.getShotParameters(getRightShooterDistanceToHub());

    //             return hood.setLeftHoodCommand(leftParameters[0]).alongWith(
    //                 hood.setRightHoodCommand(rightParameters[0]),
    //                 flywheel.setLeftFlywheelCommand(leftParameters[1]),
    //                 flywheel.setRightFlywheelCommand(rightParameters[1])
    //             );
    //         },
    //         Set.of(flywheel, hood)
    //     );
    // }

    // public Command passFuelCommand() {
    //     return new InstantCommand();
    // }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Turret Angle", getTurretShotAngle());
        // SmartDashboard.putNumber("Left Shooter Distance", getLeftShooterDistanceToHub());
        // SmartDashboard.putNumber("Right Shooter Distance", getRightShooterDistanceToHub());
    }
}