package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

import java.util.Set;
import java.util.function.BooleanSupplier;
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

    private final ShotTable shotTable = new ShotTable().backup();

    private Supplier<Translation2d> targetSupplier = () -> FieldPositions.kHub.get();
    private Supplier<Translation2d> passSupplier = () -> {
        return swerve.getCurrentPose().getY() > FieldPositions.kFieldWidth / 2.0 ? 
            FieldPositions.kLeftPass.get() : 
            FieldPositions.kRightPass.get();
    };

    private Supplier<Translation2d> appliedLeftTargetSupplier = targetSupplier;
    private Supplier<Translation2d> appliedRightTargetSupplier = targetSupplier;

    private double targetLeftHood = 0;
    private double targetRightHood = 0;
    private double targetLeftFlywheel = 0;
    private double targetRightFlywheel = 0;

    private boolean adjustTargetForMovingShots = false;

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
            pivot.deploy().alongWith(intake.setVelocity(RotationsPerSecond.of(100))),
            pivot.retract().alongWith(intake.setVelocity(RotationsPerSecond.of(75))),
            deployed
        );
    }

    public Command intake() {
        return pivot.deploy().alongWith(intake.setVelocity(RotationsPerSecond.of(100)));
    }

    public Command index(boolean withUnjam) {
        return Commands.either(
            indexer.setVelocity(() -> RotationsPerSecond.of(100))
            .withTimeout(1.5)
            .andThen(indexer.setVelocity(() -> RotationsPerSecond.of(-30)).withTimeout(0.125))
            .repeatedly(), 
            indexer.setVelocity(() -> RotationsPerSecond.of(100)), 
            () -> false
        );
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

    public Command track(Supplier<Translation2d> target) {
        return leftTurret.track(target).alongWith(rightTurret.track(target));
    }

    public Command track(Supplier<Translation2d> left, Supplier<Translation2d> right) {
        return leftTurret.track(left).alongWith(rightTurret.track(right));
    }

    public Command trackAppliedTarget() {
        return this.track(appliedLeftTargetSupplier, appliedRightTargetSupplier);
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

    public boolean turretsCanShoot() {
        return leftTurret.atTarget(3) && rightTurret.atTarget(3);
    }

    public boolean shootersReady() {
        return leftShooterReady() && rightShooterReady();
    }

    public boolean hoodAndFlywheelsReady() {
        return leftHood.atTarget() && leftFlywheel.atTargetVelocity() && rightHood.atTarget() && rightFlywheel.atTargetVelocity();
    }

    public Command shoot(Supplier<Translation2d> target) {
        return new DeferredCommand(() -> {
            this.targetSupplier = target;
            this.adjustTargetForMovingShots = true;
            Command shootCommand = new ParallelCommandGroup(
                setFlywheels(() -> targetLeftFlywheel, () -> targetRightFlywheel),
                setHoods(() -> targetLeftHood, () -> targetRightHood),
                trackAppliedTarget(),
                runIntake(() -> false),
                // Commands.waitSeconds(1.5).andThen(index(true))
                // Commands.waitUntil(() -> shootersReady()).andThen(index(true))
                (Commands.waitUntil(() -> shootersReady())
                .andThen(index(true).until(() -> !turretsCanShoot()))
                ).repeatedly()
            );

            return shootCommand;
        }, Set.of(leftFlywheel, rightFlywheel, leftHood, rightHood, leftTurret, rightTurret, indexer, pivot, intake));
    }

    public Command shoot() {
        return this.shoot(targetSupplier);
    }

    public Command feedthrough(Supplier<Translation2d> target) {
        return new DeferredCommand(() -> {
            this.targetSupplier = target;
            this.adjustTargetForMovingShots = true;
            Command shootCommand = new ParallelCommandGroup(
                setFlywheels(() -> targetLeftFlywheel, () -> targetRightFlywheel),
                setHoods(() -> targetLeftHood, () -> targetRightHood),
                track(targetSupplier),
                intake(),
                // Commands.waitSeconds(1.5).andThen(index(true))
                (Commands.waitUntil(() -> shootersReady())
                .andThen(index(true))//.until(() -> !turretsCanShoot()))
                ).repeatedly()
            );

            return shootCommand;
        }, Set.of(leftFlywheel, rightFlywheel, leftHood, rightHood, leftTurret, rightTurret, indexer, pivot, intake));
    }

    public Command feedthrough() {
        return this.feedthrough(() -> FieldPositions.kHub.get());
    }

    public Command passFeedthrough() {
        return this.feedthrough(passSupplier);
    }

    public Command autoShoot() {
        return this.shoot().withTimeout(5.0)
            .andThen(Commands.deadline(Commands.waitSeconds(0.1), stopShooters()));
    }

    public Command pass() {
        return this.forceShoot(passSupplier, 0.25);
    }

    public Command forceShoot(Supplier<Translation2d> target, double indexDelay) {
        return new DeferredCommand(() -> {
            this.targetSupplier = target;
            this.adjustTargetForMovingShots = false;

            Command shootCommand = new ParallelCommandGroup(
                setFlywheels(() -> targetLeftFlywheel, () -> targetRightFlywheel),
                setHoods(() -> targetLeftHood, () -> targetRightHood),
                track(targetSupplier),
                runIntake(() -> false),
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
                runIntake(() -> false),
                Commands.either(
                    setTurrets(() -> 0, () -> 0), 
                    setTurrets(() -> 180, () -> 180), 
                    () -> zeroTurret
                ),
                // Commands.waitSeconds(1)
                Commands.waitUntil(() -> hoodAndFlywheelsReady())
                .andThen(index(true))
            );
            return shootCommand;
        }, Set.of(leftFlywheel, rightFlywheel, leftHood, rightHood, leftTurret, rightTurret, indexer, pivot, intake));
    }

    public Command shoot(double distance, boolean zeroTurret) {
        return this.shoot(() -> distance, zeroTurret);
    }

    @Override
    public void periodic() {
        Translation2d swervePose = swerve.getCurrentPose().getTranslation();
        Translation2d leftPose = swervePose.plus(kRobotToLeftTurret.toTranslation2d().rotateBy(swerve.getCurrentPose().getRotation()));
        Translation2d rightPose = swervePose.plus(kRobotToRightTurret.toTranslation2d().rotateBy(swerve.getCurrentPose().getRotation()));

        double[] leftParameters = shotTable.getShotParameters(targetSupplier.get().minus(leftPose).getNorm());
        double[] rightParameters = shotTable.getShotParameters(targetSupplier.get().minus(rightPose).getNorm());

        ChassisSpeeds swerveSpeed = swerve.getFieldRelativeChassisSpeeds();
        Translation2d newLeftTarget = targetSupplier.get().minus(new Translation2d(swerveSpeed.vxMetersPerSecond * leftParameters[2], swerveSpeed.vyMetersPerSecond * leftParameters[2]));
        Translation2d newRightTarget = targetSupplier.get().minus(new Translation2d(swerveSpeed.vxMetersPerSecond * rightParameters[2], swerveSpeed.vyMetersPerSecond * rightParameters[2]));

        double[] newLeftParameters = shotTable.getShotParameters(leftPose.minus(newLeftTarget).getNorm());
        double[] newRightParameters = shotTable.getShotParameters(rightPose.minus(newRightTarget).getNorm());

        Translation2d newerLeftTarget = newLeftTarget.minus(new Translation2d(swerveSpeed.vxMetersPerSecond * newLeftParameters[2], swerveSpeed.vyMetersPerSecond * newLeftParameters[2]));
        Translation2d newerRightTarget = newRightTarget.minus(new Translation2d(swerveSpeed.vxMetersPerSecond * newRightParameters[2], swerveSpeed.vyMetersPerSecond * newRightParameters[2]));

        double[] newerLeftParameters = shotTable.getShotParameters(leftPose.minus(newerLeftTarget).getNorm());
        double[] newerRightParameters = shotTable.getShotParameters(rightPose.minus(newerRightTarget).getNorm());

        double[] appliedLeftParameters = adjustTargetForMovingShots ? newerLeftParameters : leftParameters;
        double[] appliedRightParameters = adjustTargetForMovingShots ? newerRightParameters : rightParameters;

        this.appliedLeftTargetSupplier = () -> adjustTargetForMovingShots ? newerLeftTarget : targetSupplier.get();
        this.appliedLeftTargetSupplier = () -> adjustTargetForMovingShots ? newerRightTarget : targetSupplier.get();

        if (Robot.isSimulation()) {
            SmartDashboard.putBoolean("SS/Shooters Ready", shootersReady());
            SmartDashboard.putBoolean("SS/Flywheel At Target", leftFlywheel.atTargetVelocity());
            SmartDashboard.putBoolean("SS/Hood At Target", leftHood.atTarget());
            SmartDashboard.putBoolean("SS/Turret At Target", leftTurret.atTarget());
            SmartDashboard.putNumber("SS/Turret Target", leftTurret.getTarget());
            SmartDashboard.putNumber("SS/Turret", leftTurret.getDegrees());

            Logger.recordOutput("SmartLogs/LeftTargetPose", new Pose2d(appliedLeftTargetSupplier.get(), new Rotation2d()));
            Logger.recordOutput("SmartLogs/RightTargetPose", new Pose2d(appliedRightTargetSupplier.get(), new Rotation2d()));
        }

        targetLeftHood = appliedLeftParameters[0];
        targetRightHood = appliedRightParameters[0];
        targetLeftFlywheel = appliedLeftParameters[1];
        targetRightFlywheel = appliedRightParameters[1];
    }
}