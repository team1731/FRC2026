package frc.robot.subsystems;

import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

import java.util.Set;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.feeder.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.shooter.ShotTable;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystemAI;

public class Superstructure extends SubsystemBase {
    private SwerveSubsystem swerve;
    private FlywheelSubsystem flywheel;
    private HoodSubsystem hood;
    private TurretSubsystemAI leftTurret, rightTurret;
    private IndexerSubsystem indexer;
    private IntakePivotSubsystem pivot;
    private IntakeRollerSubsystem intake;

    private ShotTable shotTable = new ShotTable();

    // Targets
    private final Translation2d BLUE_TARGET = new Translation2d(4.625594, 4.034536);
    private final Translation2d RED_TARGET = new Translation2d(11.915394, 4.034536);

    // private static Supplier<Pose2d> hubSupplier = () -> new Pose2d(Utils.flip(FieldConstants.Hub.topCenterPoint.toTranslation2d()), new Rotation2d());
    // private static Supplier<Pose2d> passSupplier = () -> new Pose2d(Utils.flip(new Pose2d(1, 1, new Rotation2d()).getTranslation()), new Rotation2d());

    public Superstructure(SwerveSubsystem swerve, FlywheelSubsystem flywheel, HoodSubsystem hood,  
                            IndexerSubsystem indexer, IntakePivotSubsystem pivot, IntakeRollerSubsystem intake,
                                TurretSubsystemAI leftTurret, TurretSubsystemAI rightTurret) {
        this.swerve = swerve;
        this.flywheel = flywheel;
        this.hood = hood;
        this.indexer = indexer;
        this.pivot = pivot;
        this.intake = intake;
        this.leftTurret = leftTurret;
        this.rightTurret = rightTurret;
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

    public Command warmupCommand() {
        return new ParallelCommandGroup(
            flywheel.setFlywheelVelocityCommand(50, 50)
        );
    }

    public Command shootFuelCommand(Translation2d target) {
        return new DeferredCommand(() -> {
            Translation2d swervePose = swerve.getCurrentPose().getTranslation();
            Translation2d leftPose = swervePose.plus(kLeftTurretToRobot.getTranslation().toTranslation2d().rotateBy(swerve.getCurrentPose().getRotation()));
            Translation2d rightPose = swervePose.plus(kRightTurretToRobot.getTranslation().toTranslation2d().rotateBy(swerve.getCurrentPose().getRotation()));

            double[] leftParameters = shotTable.getShotParameters(leftPose.minus(target).getNorm());
            double[] rightParameters = shotTable.getShotParameters(rightPose.minus(target).getNorm());

            ChassisSpeeds swerveSpeed = swerve.getFieldRelativeChassisSpeeds();
            Translation2d newTarget = target.minus(new Translation2d(swerveSpeed.vxMetersPerSecond * leftParameters[2], swerveSpeed.vyMetersPerSecond * rightParameters[2]));

            double[] newLeftParameters = shotTable.getShotParameters(leftPose.minus(newTarget).getNorm());
            double[] newRightParameters = shotTable.getShotParameters(rightPose.minus(newTarget).getNorm());
            
            return flywheel.setFlywheelVelocityCommand(() -> newLeftParameters[1], () -> newRightParameters[1])
            .alongWith(
                hood.setHoodCommand(() -> newLeftParameters[0], () -> newRightParameters[0]),
                leftTurret.trackTargetCommand(newTarget),
                rightTurret.trackTargetCommand(newTarget),
                pivot.retractCommand(),
                intake.setPercentOutputCommand(0.75),
                Commands.waitSeconds(1.5)
                .andThen(indexer.setPercentOutputCommand(1.0)
                .withTimeout(1.5)
                .andThen(indexer.setPercentOutputCommand(-1).withTimeout(0.125)).repeatedly())
            );
        }, Set.of(flywheel, hood, indexer, intake, pivot, leftTurret, rightTurret));
    }

    public Command shootFuelCommand(double distance) {
        return new DeferredCommand(() -> {
            double[] leftParameters = shotTable.getShotParameters(distance);
            double[] rightParameters = shotTable.getShotParameters(distance);

            return flywheel.setFlywheelVelocityCommand(() -> leftParameters[1], () -> rightParameters[1])
            .alongWith(
                hood.setHoodCommand(() -> leftParameters[0], () -> rightParameters[0]),
                leftTurret.setZeroCommand(),
                rightTurret.setZeroCommand(),
                pivot.retractCommand(),
                intake.setPercentOutputCommand(0.75),
                Commands.waitSeconds(1.5)
                .andThen(indexer.setPercentOutputCommand(1.0)
                .withTimeout(1.5)
                .andThen(indexer.setPercentOutputCommand(-1).withTimeout(0.125)).repeatedly())
            );
        }, Set.of(flywheel, hood, indexer, intake, pivot, leftTurret, rightTurret));
    }

    public Command shootAutomatedCommand() {
        return new DeferredCommand(() -> {
            Translation2d swervePose = swerve.getCurrentPose().getTranslation();
            Translation2d leftPose = swervePose.plus(kLeftTurretToRobot.getTranslation().toTranslation2d().rotateBy(swerve.getCurrentPose().getRotation()));
            Translation2d rightPose = swervePose.plus(kRightTurretToRobot.getTranslation().toTranslation2d().rotateBy(swerve.getCurrentPose().getRotation()));

            Translation2d target = Robot.isRedAlliance() ? RED_TARGET : BLUE_TARGET;

            double[] leftParameters = shotTable.getShotParameters(leftPose.minus(target).getNorm());
            double[] rightParameters = shotTable.getShotParameters(rightPose.minus(target).getNorm());

            ChassisSpeeds swerveSpeed = swerve.getFieldRelativeChassisSpeeds();
            Translation2d newTarget = target.minus(new Translation2d(swerveSpeed.vxMetersPerSecond * leftParameters[2], swerveSpeed.vyMetersPerSecond * rightParameters[2]));

            double[] newLeftParameters = shotTable.getShotParameters(leftPose.minus(newTarget).getNorm());
            double[] newRightParameters = shotTable.getShotParameters(rightPose.minus(newTarget).getNorm());
            
            return flywheel.setFlywheelVelocityCommand(() -> newLeftParameters[1], () -> newRightParameters[1])
            .alongWith(
                hood.setHoodCommand(() -> newLeftParameters[0], () -> newRightParameters[0]),
                leftTurret.trackTargetCommand(newTarget),
                rightTurret.trackTargetCommand(newTarget),
                pivot.retractCommand(),
                intake.setPercentOutputCommand(0.75),
                Commands.defer(() -> {
                    if (leftTurret.isAtTarget(3) && rightTurret.isAtTarget(3) 
                        && flywheel.atBothTargetVelocity() && hood.atBothTarget()) {
                            return indexer.setPercentOutputCommand(1d);
                        } 
                    return indexer.setPercentOutputCommand(0d);
                }, Set.of(indexer))
            );
        }, Set.of(flywheel, hood, indexer, intake, pivot, leftTurret, rightTurret));
    }

    public Command shootFuelCommand() {
        return shootFuelCommand(Robot.isRedAlliance() ? RED_TARGET : BLUE_TARGET);
    }

    public Command passFuelCommand() {
        return this.shootFuelCommand(Robot.isRedAlliance() ? new Translation2d(1, 1) : new Translation2d(16, 16));
    }

    public Command immediateShootCommand() {
        return new DeferredCommand(() -> {
            Translation2d target = Robot.isRedAlliance() ? RED_TARGET : BLUE_TARGET;
            Translation2d swervePose = swerve.getCurrentPose().getTranslation();
            Translation2d leftPose = swervePose.plus(kLeftTurretToRobot.getTranslation().toTranslation2d().rotateBy(swerve.getCurrentPose().getRotation()));
            Translation2d rightPose = swervePose.plus(kRightTurretToRobot.getTranslation().toTranslation2d().rotateBy(swerve.getCurrentPose().getRotation()));

            double[] leftParameters = shotTable.getShotParameters(leftPose.minus(target).getNorm());
            double[] rightParameters = shotTable.getShotParameters(rightPose.minus(target).getNorm());

            ChassisSpeeds swerveSpeed = swerve.getFieldRelativeChassisSpeeds();
            Translation2d newTarget = target.minus(new Translation2d(swerveSpeed.vxMetersPerSecond * leftParameters[2], swerveSpeed.vyMetersPerSecond * rightParameters[2]));

            double[] newLeftParameters = shotTable.getShotParameters(leftPose.minus(newTarget).getNorm());
            double[] newRightParameters = shotTable.getShotParameters(rightPose.minus(newTarget).getNorm());
            
            return flywheel.setFlywheelVelocityCommand(() -> newLeftParameters[1], () -> newRightParameters[1])
            .alongWith(
                hood.setHoodCommand(() -> newLeftParameters[0], () -> newRightParameters[0]),
                leftTurret.trackTargetCommand(newTarget),
                rightTurret.trackTargetCommand(newTarget),
                pivot.retractCommand(),
                intake.setPercentOutputCommand(0.75),
                Commands.waitSeconds(1.5)
                .andThen(indexer.setPercentOutputCommand(1.0)
                .withTimeout(1.5)
                .andThen(indexer.setPercentOutputCommand(-1).withTimeout(0.125)).repeatedly())
            );
        }, Set.of(flywheel, hood, indexer, intake, pivot, leftTurret, rightTurret));
    }

    public Command feedthroughFuelCommand() {
        return new DeferredCommand(() -> {
            Translation2d target = Robot.isRedAlliance() ? RED_TARGET : BLUE_TARGET;

            double[] leftParameters = shotTable.getShotParameters(swerve.getCurrentPose().transformBy(new Transform2d(kLeftTurretToRobot.getX(), kLeftTurretToRobot.getY(), swerve.getCurrentPose().getRotation())).minus(new Pose2d(target, new Rotation2d())).getTranslation().getNorm());
            double[] rightParameters = shotTable.getShotParameters(swerve.getCurrentPose().transformBy(new Transform2d(kRightTurretToRobot.getX(), kRightTurretToRobot.getY(), swerve.getCurrentPose().getRotation())).minus(new Pose2d(target, new Rotation2d())).getTranslation().getNorm());

            return flywheel.setFlywheelVelocityCommand(() -> leftParameters[1], () -> rightParameters[1])
            .alongWith(
                hood.setHoodCommand(() -> leftParameters[0], () -> rightParameters[0]),
                pivot.deployCommand(),
                intake.setPercentOutputCommand(1.0),
                // Commands.waitUntil(() -> flywheel.atBothTargetVelocity() && hood.atBothTarget() && leftTurret.isAtTarget(5) && rightTurret.isAtTarget(5))
                Commands.waitSeconds(1.5)
                .andThen(indexer.setPercentOutputCommand(1.0))
            );
        }, Set.of(flywheel, hood, indexer, intake, pivot));
    }

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

   // public Command aimTurretCommand() {
       // return turret.aimCommand(() -> swerve.getCurrentPose(), hubSupplier);
   // }

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