package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intakedeploy.IntakeDeploySubsystem;
import frc.robot.subsystems.intakeroller.IntakeRollerSubsystem;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.shooting.ShotGenerator;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * Coordinates mechanisms that need to move and interact together
 */
public class Superstructure extends SubsystemBase {
    public static final Superstructure kInstance = new Superstructure();
    private Superstructure() {}

    private ShotGenerator generator = new ShotGenerator();

    // =========================================================================
    // Helper commands
    // =========================================================================

    private boolean readyToShoot() {
        return HoodSubsystem.kInstance.atSetpoint() && FlywheelSubsystem.kInstance.atSetpoint();// && SwerveSubsystem.kInstance.lockedOnTarget();
    }

    private Command feed() {
        return IndexerSubsystem.kInstance.feed().alongWith(KickerSubsystem.kInstance.feed());
    }

    private Command applyTargetHoodAndFlywheel() {
        return HoodSubsystem.kInstance.setAngle(() -> generator.targetHoodAngle).alongWith(FlywheelSubsystem.kInstance.shoot(() -> generator.targetFlywheelVelocity));
    }

    private Command setTarget(Supplier<Translation2d> target, boolean adjustForMovingShot) {
        return new InstantCommand(() -> {
            generator.setTarget(target, adjustForMovingShot);
        });
    }

    private Command jiggleIntakeForShot() {
        return IntakeDeploySubsystem.kInstance.jiggle().alongWith(IntakeRollerSubsystem.kInstance.intake()); // intakedeploy and intakeroller should be applied here
    }

    // =========================================================================
    // Action commands
    // =========================================================================

    public Command lockSwerveToHub() {
        return SwerveSubsystem.kInstance.lockHeadingTarget(generator.kHubSupplier);
    }

    public Command squeezeHopper() {
        return HopperSubsystem.kInstance.collapse();
    }

    public Command raiseHopper() {
        return HopperSubsystem.kInstance.extend();
    }

    public Command intake() {
        return IntakeDeploySubsystem.kInstance.deploy().alongWith(IntakeRollerSubsystem.kInstance.intake());
    }

    public Command warmupWithHood() {
        return HoodSubsystem.kInstance.setAngle(() -> generator.targetHoodAngle).alongWith(FlywheelSubsystem.kInstance.shoot(() -> generator.targetFlywheelVelocity));
    }

    public Command warmupWithoutHood() {
        return HoodSubsystem.kInstance.home().alongWith(FlywheelSubsystem.kInstance.shoot(() -> generator.targetFlywheelVelocity));
    }

    private Command shoot(Supplier<Translation2d> target, boolean adjustForMovingShot, boolean trackTarget, boolean feedthrough, BooleanSupplier shotCondition) {
        return new SequentialCommandGroup(
            setTarget(target, adjustForMovingShot),
            new ParallelCommandGroup(
                SwerveSubsystem.kInstance.joystickTargetLock(target),
                applyTargetHoodAndFlywheel(),
                Commands.waitUntil(shotCondition).andThen(
                    new ParallelCommandGroup( // Only start the feeding and squeezing sequence after we are ready to shoot
                        this.feed(),
                        Commands.waitSeconds(1.5).andThen(HopperSubsystem.kInstance.collapse())
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
            FlywheelSubsystem.kInstance.shoot(RotationsPerSecond.of(targetFlywheel)),
            HoodSubsystem.kInstance.setAngle(Rotations.of(targetHood)),
            Commands.waitUntil(shotCondition).andThen(
                new ParallelCommandGroup(
                    feed(),
                    jiggleIntakeForShot(),
                    Commands.waitSeconds(1.5).andThen(squeezeHopper())
                )
            )
        );
    }

    public Command shoot(boolean feedthrough) {
        return shoot(generator.kHubSupplier, true, true, feedthrough, this::readyToShoot);
    }

    public Command pass(boolean feedthrough) {
        return shoot(generator.kPassSupplier, true, true, feedthrough, this::readyToShoot);
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
        return HoodSubsystem.kInstance.home()
            .alongWith(
                FlywheelSubsystem.kInstance.stop(), 
                IndexerSubsystem.kInstance.stop(), 
                KickerSubsystem.kInstance.stop()
            );
    }

    @Override
    public void periodic() {
        generator.update();
    }
}