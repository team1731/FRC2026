package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;

public class ShotCalculator {
    private FlywheelSubsystem flywheel;
    private HoodSubsystem hood;
    private TurretSubsystem turret;
    
    public ShotCalculator(FlywheelSubsystem flywheel, HoodSubsystem hood, TurretSubsystem turret) {
        this.flywheel = flywheel;
        this.hood = hood;
        this.turret = turret;
    }

    public Command aimAtPoseCommand(Supplier<Pose2d> currentPose, Supplier<Pose2d> targetPose) {
        return Commands.none();
    }
}