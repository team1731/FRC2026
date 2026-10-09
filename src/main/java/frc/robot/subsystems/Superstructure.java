package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.shooting.ShotGenerator;
import frc.robot.shooting.ShotTable;

/**
 * Coordinates mechanisms that need to move and interact together
 */
public class Superstructure {
    private ShotGenerator generator = new ShotGenerator();
    public Superstructure() {}


    // =========================================================================
    // Helper commands
    // =========================================================================

    private boolean readyToShoot() {
        return Robot.hood.atSetpoint() && Robot.flywheel.atSetpoint();// && SwerveSubsystem.kInstance.lockedOnTarget();
    }

    private Command feed() {
        return Robot.indexer.feed().alongWith(Robot.kicker.feed());
    }

    private Command applyTargetHoodAndFlywheel() {
        return Robot.hood.setAngle(() -> generator.targetHoodAngle).alongWith(Robot.flywheel.shoot(() -> generator.targetFlywheelVelocity));
    }

    private Command setTarget(Supplier<Translation2d> target, boolean adjustForMovingShot, ShotTable table) {
        return new InstantCommand(() -> {
            generator.setTarget(target, adjustForMovingShot, table);
        });
    }

    private Command jiggleIntakeForShot() {
        return Robot.intakedeploy.jiggle().alongWith(Robot.intakeroller.intake()); // intakedeploy and intakeroller should be applied here
    }

    // =========================================================================
    // Action commands
    // =========================================================================

    public Command lockSwerveToHub() {
        return Robot.swerve.lockHeadingTarget(generator.kHubSupplier);
    }

    public Command squeezeHopper() {
        return Robot.hopper.collapse();
    }

    public Command raiseHopper() {
        return Robot.hopper.extend();
    }

    public Command intake() {
        return Robot.intakedeploy.deploy().alongWith(Robot.intakeroller.intake());
    }

    public Command warmupWithHood() {
        return Robot.hood.setAngle(() -> generator.targetHoodAngle).alongWith(Robot.flywheel.shoot(() -> generator.targetFlywheelVelocity));
    }

    public Command warmupWithoutHood() {
        return Robot.hood.home().alongWith(Robot.flywheel.shoot(() -> generator.targetFlywheelVelocity));
    }

    private Command shoot(Supplier<Translation2d> target, boolean adjustForMovingShot, ShotTable table, boolean feedthrough, BooleanSupplier shotCondition) {
        return new SequentialCommandGroup(
            setTarget(target, adjustForMovingShot, table),
            new ParallelCommandGroup(
                Robot.swerve.joystickTargetLock(generator.appliedTargetSupplier),
                applyTargetHoodAndFlywheel(),
                Commands.waitUntil(shotCondition).andThen(
                    new ParallelCommandGroup( // Only start the feeding and squeezing sequence after we are ready to shoot
                        this.feed(),
                        Commands.waitSeconds(1.5).andThen(Robot.hopper.collapse())
                    )
                ),
                Commands.either( // If feedthrough continue intaking, otherwise jiggle
                    this.intake(),
                    Commands.waitUntil(shotCondition)
                        .andThen(jiggleIntakeForShot()),
                    () -> feedthrough
                )
            )
        );
    }

    private Command shoot(double targetFlywheel, double targetHood, BooleanSupplier shotCondition) {
        return new ParallelCommandGroup(
            Robot.flywheel.shoot(RotationsPerSecond.of(targetFlywheel)),
            Robot.hood.setAngle(Rotations.of(targetHood)),
            Commands.waitUntil(shotCondition).andThen(
                new ParallelCommandGroup(
                    feed(),
                    jiggleIntakeForShot(),
                    Commands.waitSeconds(1.5).andThen(squeezeHopper())
                )
            )
        );
    }

    public Command autoShoot() {
        return new SequentialCommandGroup(
            setTarget(generator.kHubSupplier, false, ShotTable.getScoringTable()),
            new ParallelCommandGroup(
                applyTargetHoodAndFlywheel(),
                Commands.waitUntil(this::readyToShoot).andThen(
                    new ParallelCommandGroup( // Only start the feeding and squeezing sequence after we are ready to shoot
                        this.feed(),
                        jiggleIntakeForShot(),
                        Commands.waitSeconds(1.5).andThen(Robot.hopper.collapse())
                    )
                )
            )
        );
    }

    public Command shoot(boolean feedthrough) {
        return shoot(generator.kHubSupplier, true, generator.shotTable, feedthrough, this::readyToShoot);
    }

    public Command pass(boolean feedthrough) {
        return shoot(generator.kPassSupplier, true, generator.passingTable, feedthrough, this::readyToShoot);
    }

    public Command overrideShot(double distance) {
        return shoot(generator.shotTable.getShotParameters(distance)[1], generator.shotTable.getShotParameters(distance)[0], this::readyToShoot);
    }

    public Command overrideShot(double hood, double flywheel) {
        return shoot(flywheel, hood, this::readyToShoot);
    }

    public Command hubShot() {
        return this.overrideShot(2.0);
    }

    public Command trenchShot() {
        return this.overrideShot(3.2);
    }

    public Command towerShot() {
        return this.overrideShot(4.0);
    }

    public Command stopShoot() {
        return Robot.hood.home()
            .alongWith(
                Robot.flywheel.stop(), 
                Robot.indexer.stop(), 
                Robot.kicker.stop()
            );
    }

    public void periodic() {
        generator.update();
    }
}