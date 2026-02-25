package frc.robot.subsystems;

import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.frc1731.Utils;
import frc.lib.frc1731.field.Field;
import frc.lib.frc6328.FieldConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.shooter.ShotProfile;
import frc.robot.subsystems.shooter.ShotTable;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;

public class Superstructure extends SubsystemBase {
    private SwerveSubsystem swerve;
    private FlywheelSubsystem flywheel;
    private HoodSubsystem hood;
    private TurretSubsystem turret;

    private ShotTable shotTable = new ShotTable();

    public static ShotProfile hubShotProfile = new ShotProfile(0, 0, 0, false);

    private static Supplier<Pose2d> hubSupplier = () -> new Pose2d(Utils.flip(FieldConstants.Hub.topCenterPoint.toTranslation2d()), new Rotation2d());
    private static Supplier<Pose2d> passSupplier = () -> new Pose2d(Utils.flip(new Pose2d().getTranslation()), new Rotation2d());

    public Superstructure(SwerveSubsystem swerve, FlywheelSubsystem flywheel, HoodSubsystem hood, TurretSubsystem turret) {
        this.swerve = swerve;
        this.flywheel = flywheel;
        this.hood = hood;
        this.turret = turret;
    }

    private double getTurretShotAngle() {
        Translation2d robotPose = swerve.getCurrentPose().getTranslation();
        Translation2d hubPose = Field.HUB.getPosition().toTranslation2d();
        double aimRadians = Math.atan((hubPose.getX() - robotPose.getX()) / (hubPose.getY() - robotPose.getY()));
        return Math.toDegrees(aimRadians) - swerve.getCurrentPose().getRotation().getDegrees();
    }

    private double getLeftShooterDistanceToHub() {
        Translation2d hub = FieldConstants.Hub.topCenterPoint.toTranslation2d();
        Pose3d leftShooter = turret.getLeftTurretPose(new Pose3d(swerve.getCurrentPose()));
        return hub.getDistance(leftShooter.getTranslation().toTranslation2d());
    }

    private double getRightShooterDistanceToHub() {
        Translation2d hub = FieldConstants.Hub.topCenterPoint.toTranslation2d();
        Pose3d rightShooter = turret.getRightTurretPose(new Pose3d(swerve.getCurrentPose()));
        return hub.getDistance(rightShooter.getTranslation().toTranslation2d());
    }

    public Command aimTurretAtHubCommand() {
        return turret.aimCommand(() -> swerve.getCurrentPose(), hubSupplier);
    }

    public Command aimTurretForPassCommand() {
        return turret.aimCommand(() -> swerve.getCurrentPose(), passSupplier);
    }

    public Command shootFuelCommand() {
        return new DeferredCommand(
            () -> {
                double[] leftParameters = shotTable.getShotParameters(getLeftShooterDistanceToHub());
                double[] rightParameters = shotTable.getShotParameters(getRightShooterDistanceToHub());

                return hood.setLeftHoodCommand(leftParameters[0]).alongWith(
                    hood.setRightHoodCommand(rightParameters[0]),
                    flywheel.setLeftFlywheelCommand(leftParameters[1]),
                    flywheel.setRightFlywheelCommand(rightParameters[1])
                );
            },
            Set.of(flywheel, hood)
        );
    }

    public Command passFuelCommand() {
        return new InstantCommand();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret Angle", getTurretShotAngle());
        SmartDashboard.putNumber("Left Shooter Distance", getLeftShooterDistanceToHub());
        SmartDashboard.putNumber("Right Shooter Distance", getRightShooterDistanceToHub());
    }
}