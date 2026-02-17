package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.frc1731.field.Field;
import frc.robot.commands.WaitForCommand;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.shooter.ShotProfile;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;

public class Superstructure extends SubsystemBase {
    private SwerveSubsystem swerve;
    private FlywheelSubsystem flywheel;
    private HoodSubsystem hood;
    private TurretSubsystem turret;

    public static ShotProfile hubShotProfile = new ShotProfile(0, 0, 0, false);

    public Superstructure(SwerveSubsystem swerve, FlywheelSubsystem flywheel, HoodSubsystem hood, TurretSubsystem turret) {
        this.swerve = swerve;
        this.flywheel = flywheel;
        this.hood = hood;
        this.turret = turret;
    }

    /**
     * TODO - Incorporate swerve yaw as well to accurately shoot into the goal from anywhere
     */
    private double getTurretShotAngle() {
        Translation2d robotPose = swerve.getCurrentPose().getTranslation();
        Translation2d hubPose = Field.HUB.getPosition().toTranslation2d();
        double aimRadians = Math.atan((hubPose.getX() - robotPose.getX()) / (hubPose.getY() - robotPose.getY()));
        return Math.toDegrees(aimRadians);
    }

    public Command passFuelCommand() {
        return new InstantCommand();
    }

    public Command shootFuelCommand(ShotProfile profile) {
        return flywheel.setVelocityCommand(RotationsPerSecond.of(profile.flywheelRPS))
        .alongWith(
            hood.setAngleCommand(Degrees.of(profile.hoodDegrees)),
            turret.setTurretAngleCommand(Degrees.of(profile.turretDegrees))
        ).andThen(
            Commands.either(
                new WaitForCommand(Commands.none(), () -> flywheel.atTargetVelocity()), // Replace with feeder.feedFuelCommand() once feeder added in 
                Commands.none(),
                () -> profile.shouldShoot
            )
        );
    }

    public Command shootFuelCommand(Supplier<ShotProfile> profileSupplier) {
        return flywheel.setVelocityCommand(RotationsPerSecond.of(profileSupplier.get().flywheelRPS))
        .alongWith(
            hood.setAngleCommand(Degrees.of(profileSupplier.get().hoodDegrees)),
            turret.setTurretAngleCommand(Degrees.of(profileSupplier.get().turretDegrees))
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
        SmartDashboard.putNumber("Turret Angle", getTurretShotAngle());
    }
}