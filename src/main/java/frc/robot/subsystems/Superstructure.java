package frc.robot.subsystems;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.frc1731.field.FieldPositions;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.indexer.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.shooter.flywheel.*;
import frc.robot.subsystems.shooter.hood.*;
import frc.robot.subsystems.shooter.turret.*;

import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

public class Superstructure extends SubsystemBase {
    private SwerveSubsystem swerve;
    private FlywheelSubsystem leftFlywheel, rightFlywheel;
    private TurretSubsystem leftTurret, rightTurret;
    private HoodSubsystem leftHood, rightHood;
    private IndexerSubsystem indexer;
    private IntakePivotSubsystem pivot;
    private IntakeRollerSubsystem intake;

    private ShotTable shotTable = new ShotTable();

    private Supplier<Translation2d> targetTranslation = () -> FieldPositions.kHub.get();

    private double targetLeftHood = 0;
    private double targetRightHood = 0;
    private double targetLeftFlywheel = 0;
    private double targetRightFlywheel = 0;

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

    public Command setFlywheels(DoubleSupplier left, DoubleSupplier right) {
        return leftFlywheel.setVelocity(left).alongWith(rightFlywheel.setVelocity(right));
    }

    public Command setHoods(DoubleSupplier left, DoubleSupplier right) {
        return leftHood.setRotations(left).alongWith(rightHood.setRotations(right));
    }

    public Command stowHoods() {
        return leftHood.stow().alongWith(rightHood.stow());
    }

    public Command setTurrets(DoubleSupplier left, DoubleSupplier right) {
        return leftTurret.setDegrees(left).alongWith(rightTurret.setDegrees(right));
    }

    public Command warmup() {
        return leftFlywheel.warmup().alongWith(rightFlywheel.warmup());
    }

    public Command trackHub() {
        return leftTurret.trackHub().alongWith(rightTurret.trackHub());
    }

    public Command track(Supplier<Translation2d> target) {
        return leftTurret.track(target).alongWith(rightTurret.track(target));
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

    public boolean shootersReady() {
        return leftShooterReady() && rightShooterReady();
    }

    public Command shoot(Supplier<Translation2d> target) {
        return new DeferredCommand(() -> {
            this.targetTranslation = target;
            Command shootCommand = new ParallelCommandGroup(
                setFlywheels(() -> targetLeftFlywheel, () -> targetRightFlywheel),
                setHoods(() -> targetLeftHood, () -> targetRightHood),
                track(targetTranslation),
                Commands.waitUntil(() -> shootersReady())
                .andThen(index(true))
            );

            return shootCommand;
        }, Set.of(leftFlywheel, rightFlywheel, leftHood, rightHood, leftTurret, rightTurret, indexer));
    }

    public Command shoot() {
        return this.shoot(() -> FieldPositions.kHub.get());
    }

    public Command autoShoot() {
        return this.shoot().withTimeout(5.0)
            .andThen(Commands.deadline(Commands.waitSeconds(0.1), stopShooters()));
    }

    public Command pass() {
        return this.shoot(() -> {
            return swerve.getCurrentPose().getY() > FieldPositions.kFieldWidth / 2.0 ? 
                FieldPositions.kLeftPass.get() : 
                FieldPositions.kRightPass.get();
        });
    }

    public Command forceShoot(Supplier<Translation2d> target, double indexDelay) {
        return new DeferredCommand(() -> {
            this.targetTranslation = target;

            Command shootCommand = new ParallelCommandGroup(
                setFlywheels(() -> targetLeftFlywheel, () -> targetRightFlywheel),
                setHoods(() -> targetLeftHood, () -> targetRightHood),
                track(targetTranslation),
                new WaitCommand(indexDelay).andThen(index(true))
            );

            return shootCommand;
        }, Set.of(leftFlywheel, rightFlywheel, leftHood, rightHood, leftTurret, rightTurret, indexer));
    }

    public Command forceShoot(double indexDelay) {
        return this.forceShoot(() -> FieldPositions.kHub.get(), indexDelay);
    }

    public Command shoot(double flywheel, double hood, boolean zeroTurret) {
        return new DeferredCommand(() -> {
            Command shootCommand = new ParallelCommandGroup(
                setFlywheels(() -> flywheel, () -> flywheel),
                setHoods(() -> hood, () -> hood),
                Commands.either(
                    setTurrets(() -> 0, () -> 0), 
                    setTurrets(() -> 180, () -> 180), 
                    () -> zeroTurret
                ),
                Commands.waitUntil(() -> shootersReady())
                .andThen(index(true))
            );

            return shootCommand;
        }, Set.of());
    }

    public Command shoot(DoubleSupplier distance, boolean zeroTurret) {
        return new DeferredCommand(() -> {
            double[] parameters = shotTable.getShotParameters(distance.getAsDouble());

            Command shootCommand = new ParallelCommandGroup(
                setFlywheels(() -> parameters[1], () -> parameters[1]),
                setHoods(() -> parameters[0], () -> parameters[0]),
                Commands.either(
                    setTurrets(() -> 0, () -> 0), 
                    setTurrets(() -> 180, () -> 180), 
                    () -> zeroTurret
                ),
                Commands.waitUntil(() -> shootersReady())
                .andThen(index(true))
            );
            return shootCommand;
        }, Set.of(leftFlywheel, rightFlywheel, leftHood, rightHood, leftTurret, rightTurret, indexer));
    }

    public Command shoot(double distance, boolean zeroTurret) {
        return this.shoot(() -> distance, zeroTurret);
    }

    @Override
    public void periodic() {
        Translation2d swervePose = swerve.getCurrentPose().getTranslation();
        Translation2d leftPose = swervePose.plus(kRobotToLeftTurret.toTranslation2d().rotateBy(swerve.getCurrentPose().getRotation()));
        Translation2d rightPose = swervePose.plus(kRobotToRightTurret.toTranslation2d().rotateBy(swerve.getCurrentPose().getRotation()));

        double[] leftParameters = shotTable.getShotParameters(leftPose.minus(targetTranslation.get()).getNorm());
        double[] rightParameters = shotTable.getShotParameters(rightPose.minus(targetTranslation.get()).getNorm());

        ChassisSpeeds swerveSpeed = swerve.getFieldRelativeChassisSpeeds();
        Translation2d newTarget = targetTranslation.get().minus(new Translation2d(swerveSpeed.vxMetersPerSecond * leftParameters[2], swerveSpeed.vyMetersPerSecond * rightParameters[2]));

        double[] newLeftParameters = shotTable.getShotParameters(leftPose.minus(newTarget).getNorm());
        double[] newRightParameters = shotTable.getShotParameters(rightPose.minus(newTarget).getNorm());

        Logger.recordOutput("SmartLogs/ShotTarget", new Pose2d(newTarget, new Rotation2d()));

        targetLeftHood = newLeftParameters[0];
        targetRightHood = newRightParameters[0];
        targetLeftFlywheel = newLeftParameters[1];
        targetRightFlywheel = newRightParameters[1];
    }
}