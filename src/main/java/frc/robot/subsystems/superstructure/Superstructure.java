package frc.robot.subsystems.superstructure;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.WaitForCommand;
import frc.robot.subsystems.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class Superstructure extends SubsystemBase {
    private FlywheelSubsystem flywheel;
    private HoodSubsystem hood;
    private TurretSubsystem turret;

    public static ShotProfile hubShotProfile = new ShotProfile(0, 0, 0, false);

    private static Pose2d shotTargetPose = new Pose2d();

    public Superstructure(FlywheelSubsystem flywheel, HoodSubsystem hood, TurretSubsystem turret) {
        this.flywheel = flywheel;
        this.hood = hood;
        this.turret = turret;
    }

    public Command passFuelCommand() {
        return new InstantCommand();
    }

    public Command shootFuelCommand(ShotProfile profile) {
        return flywheel.setVelocityCommand(profile.flywheelRPS)
        .alongWith(
            hood.setDegreesCommand(profile.hoodDegrees),
            turret.setDegreesCommand(profile.turretDegrees)
        ).andThen(
            Commands.either(
                new WaitForCommand(Commands.none(), () -> flywheel.atTargetVelocity()), // Replace with feeder.feedFuelCommand() once feeder added in 
                Commands.none(),
                () -> profile.shouldShoot
            )
        );
    }

    public Command shootFuelCommand(Supplier<ShotProfile> profileSupplier) {
        return flywheel.setVelocityCommand(profileSupplier.get().flywheelRPS)
        .alongWith(
            hood.setDegreesCommand(profileSupplier.get().hoodDegrees),
            turret.setDegreesCommand(profileSupplier.get().turretDegrees)
        ).andThen(
            Commands.either(
                new WaitForCommand(Commands.none(), () -> flywheel.atTargetVelocity()), // Replace with feeder.feedFuelCommand() once feeder added in 
                Commands.none(),
                () -> profileSupplier.get().shouldShoot
            )
        );
    }

    @Override
    public void periodic() {

    }

}