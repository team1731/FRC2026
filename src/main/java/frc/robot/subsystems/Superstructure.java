package frc.robot.subsystems;

import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.*;
import static frc.robot.subsystems.shooter.hood.HoodConstants.*;
import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.frc1731.Utils;
import frc.lib.frc6328.FieldConstants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.indexer.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.shooter.flywheel.*;
import frc.robot.subsystems.shooter.hood.*;
import frc.robot.subsystems.shooter.turret.*;

public class Superstructure {
    private SwerveSubsystem swerve;
    private FlywheelSubsystem leftFlywheel, rightFlywheel;
    private TurretSubsystem leftTurret, rightTurret;
    private HoodSubsystem leftHood, rightHood;
    private IndexerSubsystem indexer;
    private IntakePivotSubsystem pivot;
    private IntakeRollerSubsystem intake;

    private ShotTable shotTable = new ShotTable();

    // Targets
    // private final Translation2d BLUE_TARGET = new Translation2d(4.625594, 4.034536);
    // private final Translation2d RED_TARGET = new Translation2d(11.915394, 4.034536);

    private static Supplier<Translation2d> hubSupplier = () -> Utils.flip(FieldConstants.Hub.topCenterPoint.toTranslation2d());
    // private static Supplier<Pose2d> passSupplier = () -> new Pose2d(Utils.flip(new Pose2d(1, 1, new Rotation2d()).getTranslation()), new Rotation2d());

    public Superstructure(SwerveSubsystem swerve, FlywheelSubsystem leftFlywheel, FlywheelSubsystem rightFlywheel, 
                            HoodSubsystem leftHood, HoodSubsystem rightHood, IndexerSubsystem indexer, 
                            IntakePivotSubsystem pivot, IntakeRollerSubsystem intake, TurretSubsystem leftTurret, 
                            TurretSubsystem rightTurret) {
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

    public Command resetGyro() {
        return new InstantCommand(() -> swerve.resetHeadingButtonPressed());
    }

    public Command stowIntake() {
        return pivot.retract().alongWith(intake.stop());
    }

    public Command runIntake(BooleanSupplier deployed) {
        return Commands.either(
            pivot.deploy().alongWith(intake.setPercentOutput(1d)), 
            pivot.retract().alongWith(intake.setPercentOutput(0.25)), 
            deployed
        );
    }

    public Command index(boolean withUnjam) {
        return Commands.either(
            indexer.setPercentOutput(1.0)
            .withTimeout(1.5)
            .andThen(indexer.setPercentOutput(-1.0).withTimeout(0.125))
            .repeatedly(), 
            indexer.setPercentOutput(1.0), 
            () -> withUnjam
        );
    }

    public Command reverseFeeder() {
        return indexer.setPercentOutput(-1);
    }

    public Command setFlywheels(double left, double right) {
        return leftFlywheel.setVelocity(left).alongWith(rightFlywheel.setVelocity(right));
    }

    public Command setHoods(double left, double right) {
        return leftHood.setRotations(left).alongWith(rightHood.setRotations(right));
    }

    public Command stowHoods() {
        return setHoods(kMinRotations, kMinRotations);
    }

    public Command setTurrets(double left, double right) {
        return leftTurret.setDegrees(left).alongWith(rightTurret.setDegrees(right));
    }

    public Command warmup() {
        return setFlywheels(kWarmupVelocity, kWarmupVelocity);
    }

    public Command trackHub() {
        return leftTurret.trackHub().alongWith(rightTurret.trackHub());
    }

    public Command stopShooters() {
        return leftFlywheel.stopOnce().andThen(rightFlywheel.stopOnce());
    }

    public boolean leftShooterReady() {
        return leftHood.atTarget() && leftFlywheel.atTargetVelocity()  && leftTurret.atTarget();
    }

    public boolean rightShooterReady() {
        return rightHood.atTarget() && rightFlywheel.atTargetVelocity() && rightTurret.atTarget();
    }
    
    public Command wrapShot(Command shotCommand, boolean feedthrough) {
        return shotCommand.alongWith(Commands.either(runIntake(() -> true), runIntake(() -> false), () -> feedthrough));
    }

    public Command shoot(Supplier<Translation2d> target) {
        return new DeferredCommand(() -> {
            Translation2d swervePose = swerve.getCurrentPose().getTranslation();
            Translation2d leftPose = swervePose.plus(kLeftTurretToRobot.getTranslation().toTranslation2d().rotateBy(swerve.getCurrentPose().getRotation()));
            Translation2d rightPose = swervePose.plus(kRightTurretToRobot.getTranslation().toTranslation2d().rotateBy(swerve.getCurrentPose().getRotation()));

            double[] leftParameters = shotTable.getShotParameters(leftPose.minus(target.get()).getNorm());
            double[] rightParameters = shotTable.getShotParameters(rightPose.minus(target.get()).getNorm());

            ChassisSpeeds swerveSpeed = swerve.getFieldRelativeChassisSpeeds();
            Translation2d newTarget = target.get().minus(new Translation2d(swerveSpeed.vxMetersPerSecond * leftParameters[2], swerveSpeed.vyMetersPerSecond * rightParameters[2]));

            double[] newLeftParameters = shotTable.getShotParameters(leftPose.minus(newTarget).getNorm());
            double[] newRightParameters = shotTable.getShotParameters(rightPose.minus(newTarget).getNorm());

            Command shootCommand = new ParallelCommandGroup(
                setFlywheels(newLeftParameters[1], newRightParameters[1]),
                setHoods(newLeftParameters[0], newRightParameters[0]),
                trackHub(),
                Commands.waitUntil(() -> leftShooterReady() && rightShooterReady())
                .andThen(index(true))
            );

            // if (leftShooterReady() && rightShooterReady()) shootCommand.alongWith(index(true));
            return shootCommand;
        }, Set.of(leftFlywheel, rightFlywheel, leftHood, rightHood, leftTurret, rightTurret, indexer));
    }

    public Command shoot() {
        return this.shoot(hubSupplier);
    }

    public Command pass() {
        return this.shoot(() -> {
            double targetX = 1;
            double targetY = 1;

            if (Robot.isRedAlliance()) targetX = FieldConstants.fieldLength - targetX;
            if (swerve.getCurrentPose().getY() > FieldConstants.fieldWidth / 2.0) targetY = FieldConstants.fieldWidth - targetY;

            return new Translation2d(targetX, targetY);
        });
    }

    public Command forceShoot(Supplier<Translation2d> target, double indexDelay) {
        return new DeferredCommand(() -> {
            Translation2d swervePose = swerve.getCurrentPose().getTranslation();
            Translation2d leftPose = swervePose.plus(kLeftTurretToRobot.getTranslation().toTranslation2d().rotateBy(swerve.getCurrentPose().getRotation()));
            Translation2d rightPose = swervePose.plus(kRightTurretToRobot.getTranslation().toTranslation2d().rotateBy(swerve.getCurrentPose().getRotation()));

            double[] leftParameters = shotTable.getShotParameters(leftPose.minus(target.get()).getNorm());
            double[] rightParameters = shotTable.getShotParameters(rightPose.minus(target.get()).getNorm());

            ChassisSpeeds swerveSpeed = swerve.getFieldRelativeChassisSpeeds();
            Translation2d newTarget = target.get().minus(new Translation2d(swerveSpeed.vxMetersPerSecond * leftParameters[2], swerveSpeed.vyMetersPerSecond * rightParameters[2]));

            double[] newLeftParameters = shotTable.getShotParameters(leftPose.minus(newTarget).getNorm());
            double[] newRightParameters = shotTable.getShotParameters(rightPose.minus(newTarget).getNorm());

            Command shootCommand = new ParallelCommandGroup(
                setFlywheels(newLeftParameters[1], newRightParameters[1]),
                setHoods(newLeftParameters[0], newRightParameters[0]),
                trackHub(),
                new WaitCommand(indexDelay).andThen(index(true))
            );

            return shootCommand;
        }, Set.of(leftFlywheel, rightFlywheel, leftHood, rightHood, leftTurret, rightTurret, indexer));
    }

    public Command forceShoot(double indexDelay) {
        return this.forceShoot(hubSupplier, indexDelay);
    }

    public Command shoot(double flywheel, double hood, boolean zeroTurret) {
        return new DeferredCommand(() -> {
            Command shootCommand = new ParallelCommandGroup(
                setFlywheels(flywheel, flywheel),
                setHoods(hood, hood),
                trackHub()
            );

            if (leftShooterReady() && rightShooterReady()) shootCommand.alongWith(index(true));
            return shootCommand;
        }, Set.of());
    }

    public Command shoot(DoubleSupplier distance, boolean zeroTurret) {
        return new DeferredCommand(() -> {
            double[] parameters = shotTable.getShotParameters(distance.getAsDouble());

            Command shootCommand = new ParallelCommandGroup(
                setFlywheels(parameters[1], parameters[1]),
                setHoods(parameters[0], parameters[0]),
                Commands.either(
                    setTurrets(0, 0), 
                    setTurrets(180, 180), 
                    () -> zeroTurret
                )
            );

            if (leftShooterReady() && rightShooterReady()) shootCommand.alongWith(index(true));
            return shootCommand;
        }, Set.of(leftFlywheel, rightFlywheel, leftHood, rightHood, leftTurret, rightTurret, indexer));
    }

    public Command shoot(double distance, boolean zeroTurret) {
        return this.shoot(() -> distance, zeroTurret);
    }

    public Command autoShoot() {
        return (forceShoot(1.25).alongWith(runIntake(() -> false))).withTimeout(5).andThen(setFlywheels(0d, 0d));
    }

    // public Command shootFuelCommand(Translation2d target) {
    //     return new DeferredCommand(() -> {
    //         Translation2d swervePose = swerve.getCurrentPose().getTranslation();
    //         Translation2d leftPose = swervePose.plus(kLeftTurretToRobot.getTranslation().toTranslation2d().rotateBy(swerve.getCurrentPose().getRotation()));
    //         Translation2d rightPose = swervePose.plus(kRightTurretToRobot.getTranslation().toTranslation2d().rotateBy(swerve.getCurrentPose().getRotation()));

    //         double[] leftParameters = shotTable.getShotParameters(leftPose.minus(target).getNorm());
    //         double[] rightParameters = shotTable.getShotParameters(rightPose.minus(target).getNorm());

    //         ChassisSpeeds swerveSpeed = swerve.getFieldRelativeChassisSpeeds();
    //         Translation2d newTarget = target.minus(new Translation2d(swerveSpeed.vxMetersPerSecond * leftParameters[2], swerveSpeed.vyMetersPerSecond * rightParameters[2]));

    //         double[] newLeftParameters = shotTable.getShotParameters(leftPose.minus(newTarget).getNorm());
    //         double[] newRightParameters = shotTable.getShotParameters(rightPose.minus(newTarget).getNorm());
            
    //         return flywheel.setFlywheelVelocityCommand(() -> newLeftParameters[1], () -> newRightParameters[1])
    //         .alongWith(
    //             hood.setHoodCommand(() -> newLeftParameters[0], () -> newRightParameters[0]),
    //             leftTurret.track(() -> newTarget),
    //             rightTurret.track(() -> newTarget),
    //             pivot.retractCommand(),
    //             intake.setPercentOutputCommand(0.75),
    //             Commands.waitSeconds(1.5)
    //             .andThen(runIndexerCommand())
    //         );
    //     }, Set.of(leftFlywheel, rightFlywheel, hood, indexer, intake, pivot, leftTurret, rightTurret));
    // }

    // public Command shootFuelCommand(double distance) {
    //     return new DeferredCommand(() -> {
    //         double[] leftParameters = shotTable.getShotParameters(distance);
    //         double[] rightParameters = shotTable.getShotParameters(distance);

    //         return flywheel.setFlywheelVelocityCommand(() -> leftParameters[1], () -> rightParameters[1])
    //         .alongWith(
    //             hood.setHoodCommand(() -> leftParameters[0], () -> rightParameters[0]),
    //             leftTurret.setDegrees(0),
    //             rightTurret.setDegrees(0),
    //             pivot.retractCommand(),
    //             intake.setPercentOutputCommand(0.75),
    //             Commands.waitSeconds(1.5)
    //             .andThen(runIndexerCommand())
    //         );
    //     }, Set.of(flywheel, hood, indexer, intake, pivot, leftTurret, rightTurret));
    // }

    // public Command shootFuelCommand(DoubleSupplier distance) {
    //     return new DeferredCommand(() -> {
    //         double[] leftParameters = shotTable.getShotParameters(distance.getAsDouble());
    //         double[] rightParameters = shotTable.getShotParameters(distance.getAsDouble());

    //         return flywheel.setFlywheelVelocityCommand(() -> leftParameters[1], () -> rightParameters[1])
    //         .alongWith(
    //             hood.setHoodCommand(() -> leftParameters[0], () -> rightParameters[0]),
    //             leftTurret.setDegrees(0),
    //             rightTurret.setDegrees(0),
    //             pivot.retractCommand(),
    //             intake.setPercentOutputCommand(0.75),
    //             Commands.waitSeconds(1.5)
    //             .andThen(runIndexerCommand())
    //         );
    //     }, Set.of(flywheel, hood, indexer, intake, pivot, leftTurret, rightTurret));
    // }

    // public Command shootFuelCommand(double flywheelRPS, double hoodRotations, boolean zero) {
    //     return new DeferredCommand(() -> {
    //         return this.flywheel.setFlywheelVelocityCommand(() -> flywheelRPS, () -> flywheelRPS)
    //         .alongWith(
    //             hood.setHoodCommand(() -> hoodRotations, () -> hoodRotations),
    //             Commands.either(leftTurret.setDegrees(0), leftTurret.setDegrees(180), () -> zero),
    //             Commands.either(rightTurret.setDegrees(0), rightTurret.setDegrees(180), () -> zero),
    //             pivot.retractCommand(),
    //             intake.setPercentOutputCommand(0.75),
    //             Commands.waitSeconds(1.5)
    //             .andThen(runIndexerCommand())
    //         );
    //     }, Set.of(flywheel, hood, indexer, intake, pivot, leftTurret, rightTurret));
    // }

    // public Command shootAutomatedCommand() {
    //     return new DeferredCommand(() -> {
    //         Translation2d swervePose = swerve.getCurrentPose().getTranslation();
    //         Translation2d leftPose = swervePose.plus(kLeftTurretToRobot.getTranslation().toTranslation2d().rotateBy(swerve.getCurrentPose().getRotation()));
    //         Translation2d rightPose = swervePose.plus(kRightTurretToRobot.getTranslation().toTranslation2d().rotateBy(swerve.getCurrentPose().getRotation()));

    //         Translation2d target = Robot.isRedAlliance() ? RED_TARGET : BLUE_TARGET;

    //         double[] leftParameters = shotTable.getShotParameters(leftPose.minus(target).getNorm());
    //         double[] rightParameters = shotTable.getShotParameters(rightPose.minus(target).getNorm());

    //         ChassisSpeeds swerveSpeed = swerve.getFieldRelativeChassisSpeeds();
    //         Translation2d newTarget = target.minus(new Translation2d(swerveSpeed.vxMetersPerSecond * leftParameters[2], swerveSpeed.vyMetersPerSecond * rightParameters[2]));

    //         double[] newLeftParameters = shotTable.getShotParameters(leftPose.minus(newTarget).getNorm());
    //         double[] newRightParameters = shotTable.getShotParameters(rightPose.minus(newTarget).getNorm());
            
    //         return flywheel.setFlywheelVelocityCommand(() -> newLeftParameters[1], () -> newRightParameters[1])
    //         .alongWith(
    //             hood.setHoodCommand(() -> newLeftParameters[0], () -> newRightParameters[0]),
    //             leftTurret.track(() -> target),
    //             rightTurret.track(() -> target),
    //             pivot.retractCommand(),
    //             intake.setPercentOutputCommand(0.75),
    //             Commands.defer(() -> {
    //                 if (leftTurret.atTarget() && rightTurret.atTarget()
    //                     && flywheel.atBothTargetVelocity() && hood.atBothTarget()) {
    //                         return indexer.setPercentOutputCommand(1d);
    //                     } 
    //                 return indexer.setPercentOutputCommand(0d);
    //             }, Set.of(indexer))
    //         );
    //     }, Set.of(flywheel, hood, indexer, intake, pivot, leftTurret, rightTurret));
    // }

    // public Command shootFuelCommand() {
    //     return shootFuelCommand(Robot.isRedAlliance() ? RED_TARGET : BLUE_TARGET);
    // }

    // public Command passFuelCommand() {
    //     return this.shootFuelCommand(Robot.isRedAlliance() ? new Translation2d(1, 1) : new Translation2d(16, 16));
    // }

    // public Command passFeedthroughCommand() {
    //     return this.feedthroughFuelCommand(Robot.isRedAlliance() ? new Translation2d(1, 1) : new Translation2d(16, 16));
    // }

    // public Command immediateShootCommand() {
    //     return new DeferredCommand(() -> {
    //         Translation2d target = Robot.isRedAlliance() ? RED_TARGET : BLUE_TARGET;
    //         Translation2d swervePose = swerve.getCurrentPose().getTranslation();
    //         Translation2d leftPose = swervePose.plus(kLeftTurretToRobot.getTranslation().toTranslation2d().rotateBy(swerve.getCurrentPose().getRotation()));
    //         Translation2d rightPose = swervePose.plus(kRightTurretToRobot.getTranslation().toTranslation2d().rotateBy(swerve.getCurrentPose().getRotation()));

    //         double[] leftParameters = shotTable.getShotParameters(leftPose.minus(target).getNorm());
    //         double[] rightParameters = shotTable.getShotParameters(rightPose.minus(target).getNorm());

    //         ChassisSpeeds swerveSpeed = swerve.getFieldRelativeChassisSpeeds();
    //         Translation2d newTarget = target.minus(new Translation2d(swerveSpeed.vxMetersPerSecond * leftParameters[2], swerveSpeed.vyMetersPerSecond * rightParameters[2]));

    //         double[] newLeftParameters = shotTable.getShotParameters(leftPose.minus(newTarget).getNorm());
    //         double[] newRightParameters = shotTable.getShotParameters(rightPose.minus(newTarget).getNorm());
            
    //         return flywheel.setFlywheelVelocityCommand(() -> newLeftParameters[1], () -> newRightParameters[1])
    //         .alongWith(
    //             hood.setHoodCommand(() -> newLeftParameters[0], () -> newRightParameters[0]),
    //             leftTurret.track(newTarget),
    //             rightTurret.track(newTarget),
    //             pivot.retractCommand(),
    //             intake.setPercentOutputCommand(0.75),
    //             Commands.waitSeconds(1.25)
    //             .andThen(runIndexerCommand())
    //         );
    //     }, Set.of(flywheel, hood, indexer, intake, pivot, leftTurret, rightTurret));
    // }

    // public Command feedthroughFuelCommand() {
    //     return new DeferredCommand(() -> {
    //         Translation2d target = Robot.isRedAlliance() ? RED_TARGET : BLUE_TARGET;

    //         double[] leftParameters = shotTable.getShotParameters(swerve.getCurrentPose().transformBy(new Transform2d(kLeftTurretToRobot.getX(), kLeftTurretToRobot.getY(), swerve.getCurrentPose().getRotation())).minus(new Pose2d(target, new Rotation2d())).getTranslation().getNorm());
    //         double[] rightParameters = shotTable.getShotParameters(swerve.getCurrentPose().transformBy(new Transform2d(kRightTurretToRobot.getX(), kRightTurretToRobot.getY(), swerve.getCurrentPose().getRotation())).minus(new Pose2d(target, new Rotation2d())).getTranslation().getNorm());

    //         return flywheel.setFlywheelVelocityCommand(() -> leftParameters[1], () -> rightParameters[1])
    //         .alongWith(
    //             hood.setHoodCommand(() -> leftParameters[0], () -> rightParameters[0]),
    //             pivot.deployCommand(),
    //             intake.setPercentOutputCommand(1.0),
    //             Commands.waitSeconds(1.5)
    //             .andThen(runIndexerCommand())
    //         );
    //     }, Set.of(flywheel, hood, indexer, intake, pivot));
    // }

    // public Command feedthroughFuelCommand(Translation2d target) {
    //     return new DeferredCommand(() -> {
    //         double[] leftParameters = shotTable.getShotParameters(swerve.getCurrentPose().transformBy(new Transform2d(kLeftTurretToRobot.getX(), kLeftTurretToRobot.getY(), swerve.getCurrentPose().getRotation())).minus(new Pose2d(target, new Rotation2d())).getTranslation().getNorm());
    //         double[] rightParameters = shotTable.getShotParameters(swerve.getCurrentPose().transformBy(new Transform2d(kRightTurretToRobot.getX(), kRightTurretToRobot.getY(), swerve.getCurrentPose().getRotation())).minus(new Pose2d(target, new Rotation2d())).getTranslation().getNorm());

    //         return flywheel.setFlywheelVelocityCommand(() -> leftParameters[1], () -> rightParameters[1])
    //         .alongWith(
    //             hood.setHoodCommand(() -> leftParameters[0], () -> rightParameters[0]),
    //             pivot.deployCommand(),
    //             intake.setPercentOutputCommand(1.0),
    //             Commands.waitSeconds(1.5)
    //             .andThen(runIndexerCommand())
    //         );
    //     }, Set.of(flywheel, hood, indexer, intake, pivot));
    // }
}