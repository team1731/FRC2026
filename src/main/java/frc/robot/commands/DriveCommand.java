package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DriveCommand extends Command {
    private double snailModeScalar = 1.0;
    private double targetError = 0;
    private Supplier<Translation2d> targetSupplier;
    private boolean lockToTarget = false;

    public DriveCommand(SwerveSubsystem swerve) {
        addRequirements(swerve);
    }

    public Command setSnailDrive(boolean snail) {
        return new InstantCommand(() -> snailModeScalar = snail ? SwerveConstants.kSnailDriveScalar : 1.0);
    }

    public Command lockToTarget(Supplier<Translation2d> target) {
        return new InstantCommand(() -> {
            this.lockToTarget = true;
            this.targetSupplier = target;
        })
        .finallyDo(() -> this.lockToTarget = false);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() { return false; }
}