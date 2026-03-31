package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.*;
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

    private final Supplier<Translation2d> kHubSupplier = () -> Robot.isRedAlliance() ? new Translation2d(11.915394, 4.034536) : new Translation2d(4.625594, 4.034536);
    private final Supplier<Translation2d> kPassSupplier = () -> {
        double x = 2;
        double y = 1;
        if (Robot.isRedAlliance()) {
            if (swerve.getCurrentPose().getY() > FieldPositions.kFieldWidth / 2.0) {
                y = 7;
            }
        }

        return new Translation2d(x, y);
    };

    private Supplier<Translation2d> targetSupplier = kHubSupplier;
    public Supplier<Translation2d> appliedTargetSupplier = targetSupplier;

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

    public Command collapseIntakeForScore() {
        return pivot.setManual(0.05).alongWith(intake.setPercentOutput(1.0));
    }

    public Command reverseFeeder() {
        return indexer.unjam();
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

    public Command stowHoodsOnce() {
        return leftHood.stowOnce().alongWith(rightHood.stowOnce());
    }

    public Command setTurrets(DoubleSupplier left, DoubleSupplier right) {
        return leftTurret.setDegrees(left).alongWith(rightTurret.setDegrees(right));
    }

    public Command warmup() {
        return leftFlywheel.warmup().alongWith(rightFlywheel.warmup(), leftTurret.trackHub(), rightTurret.trackHub(), leftHood.stow(), rightHood.stow());
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
        return this.track(appliedTargetSupplier, appliedTargetSupplier);
    }

    public Command stopShooters() {
        return leftFlywheel.stop().alongWith(rightFlywheel.stop(), leftHood.stow(), rightHood.stow(), indexer.stop());
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
            // this.adjustTargetForMovingShots = true;
            Command shootCommand = new ParallelCommandGroup(
                setFlywheels(() -> targetLeftFlywheel, () -> targetRightFlywheel),
                setHoods(() -> targetLeftHood, () -> targetRightHood),
                // trackAppliedTarget(),
                track(targetSupplier),
                // Commands.waitSeconds(1.5).andThen(index(true))
                // Commands.waitUntil(() -> shootersReady()).andThen(index(true))
                (Commands.waitUntil(() -> shootersReady())
                .andThen(
                    indexer.index()
                        .until(() -> !turretsCanShoot())
                        .alongWith(runIntake(() -> false))
                )).repeatedly()
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
            // this.adjustTargetForMovingShots = true;
            Command shootCommand = new ParallelCommandGroup(
                setFlywheels(() -> targetLeftFlywheel, () -> targetRightFlywheel),
                setHoods(() -> targetLeftHood, () -> targetRightHood),
                track(targetSupplier),
                intake(),
                (Commands.waitUntil(() -> shootersReady())
                .andThen(indexer.index())
                ).repeatedly()
            );

            return shootCommand;
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
        return this.forceShoot(kPassSupplier, 0.25);
    }

    public Command forceShoot(Supplier<Translation2d> target, double indexDelay) {
        return new DeferredCommand(() -> {
            this.targetSupplier = target;
            this.adjustTargetForMovingShots = false;

            Command shootCommand = new ParallelCommandGroup(
                setFlywheels(() -> targetLeftFlywheel, () -> targetRightFlywheel),
                setHoods(() -> targetLeftHood, () -> targetRightHood),
                track(targetSupplier),
                new WaitCommand(indexDelay).andThen(indexer.index().alongWith(runIntake(() -> false)))
            );

            return shootCommand;
        }, Set.of(leftFlywheel, rightFlywheel, leftHood, rightHood, leftTurret, rightTurret, indexer));
    }

    public Command forceShoot(double indexDelay) {
        return this.forceShoot(() -> FieldPositions.kHub.get(), indexDelay);
    }

    public Command manualShot(DoubleSupplier distance) {
        return new DeferredCommand(() -> {
            double[] parameters = shotTable.getShotParameters(distance.getAsDouble());

            Command shootCommand = new ParallelCommandGroup(
                setFlywheels(() -> parameters[1], () -> parameters[1]),
                setHoods(() -> parameters[0], () -> parameters[0]),
                setTurrets(() -> 0, () -> 0), 
                Commands.waitSeconds(1.0)
                .andThen(indexer.index().alongWith(runIntake(() -> false)))
            );
            return shootCommand;
        }, Set.of(leftFlywheel, rightFlywheel, leftHood, rightHood, leftTurret, rightTurret, indexer, pivot, intake));
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
                .andThen(indexer.index().alongWith(runIntake(() -> false)))
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
                // Commands.waitSeconds(1)
                Commands.waitUntil(() -> hoodAndFlywheelsReady())
                .andThen(indexer.index().alongWith(runIntake(() -> false)))
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

        // if (Robot.isSimulation()) {
        //     Logger.recordOutput("SmartLogs/TargetPose", new Pose2d(appliedTargetSupplier.get(), new Rotation2d()));
        // }

        // SmartDashboard.putNumber("Left Turret Degrees", leftTurret.getDegrees());
        // SmartDashboard.putNumber("Right Turret Degrees", rightTurret.getDegrees());

        targetLeftHood = leftParameters[0];
        targetRightHood = rightParameters[0];
        targetLeftFlywheel = leftParameters[1];
        targetRightFlywheel = rightParameters[1];
    }
}