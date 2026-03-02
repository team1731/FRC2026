package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;
import frc.robot.subsystems.vision.QuestNavSubsystem;

public class RobotContainer {
    /* Subsystems */
    protected static QuestNavSubsystem vslam;

    protected static SwerveSubsystem swerve;
    protected static FlywheelSubsystem flywheel;
    protected static HoodSubsystem hood;
    protected static TurretSubsystem turret;
    protected static IndexerSubsystem indexer;
    protected static IntakeRollerSubsystem intake;
    protected static IntakePivotSubsystem pivot;
    protected static LEDSubsystem led;

    protected static Superstructure superstructure;

    /* Driver Buttons */
    private final CommandXboxController driver = new CommandXboxController(0);
    private final Trigger dResetSwerve = driver.povRight();
    private final Trigger dShoot = driver.rightTrigger();
    private final Trigger dIntake = driver.leftTrigger();
    private final Trigger dFeedthrough = driver.leftTrigger().and(driver.rightTrigger());

    // private static final LoggedTunableNumber flywheelSetpoint = new LoggedTunableNumber("FlywheelSetpoint", () -> true);
    // private static final LoggedTunableNumber hoodSetpoint = new LoggedTunableNumber("HoodAngle", () -> true);

    /* Operator Buttons */

    public RobotContainer() {
        configureSubsystems();
        configureNamedCommands();
        configureButtonBindings();
    }

    /**
     * Configure all active subsystems on the robot and set default commands
     */
    private void configureSubsystems() {
        swerve = new SwerveSubsystem(true);
        flywheel = new FlywheelSubsystem(true);
        hood = new HoodSubsystem(true);
        turret = new TurretSubsystem(true);
        indexer = new IndexerSubsystem(true);
        pivot = new IntakePivotSubsystem(true);
        intake = new IntakeRollerSubsystem(true);
        led = new LEDSubsystem(false);

        superstructure = new Superstructure(swerve, flywheel, hood, turret, indexer, pivot, intake);

        // Drivetrain will execute this command periodically 
        // if no other command is active on the drivetrain
        swerve.setDefaultCommand(swerve.drive(driver, () -> true));

        flywheel.setDefaultCommand(flywheel.stop());
        intake.setDefaultCommand(intake.stop());
        indexer.setDefaultCommand(indexer.stop());
        hood.setDefaultCommand(hood.stow());
        turret.setDefaultCommand(superstructure.aimTurret());
        // turret.setDefaultCommand(superstructure.aimTurret());
        // turret.setDefaultCommand(turret.setRightTurretCommand(() -> -swerve.getYaw() % 360d));
    }

    private void configureNamedCommands() {
        // Named commands useful for PathPlanner events
        // ex. NamedCommands.registerCommand("Example", new ExampleCommand());
        NamedCommands.registerCommand("Warmup", superstructure.warmup());
        NamedCommands.registerCommand("Shoot", superstructure.shoot());
        NamedCommands.registerCommand("Intake", superstructure.intake());
        NamedCommands.registerCommand("Feedthrough", superstructure.feedthrough());
        NamedCommands.registerCommand("Pass", superstructure.pass());
        NamedCommands.registerCommand("Stow Hood", hood.stow());
        NamedCommands.registerCommand("Init Climb", Commands.print("Starting Climb!!!"));
        NamedCommands.registerCommand("Climb", Commands.print("Climbing!!!"));
    }

    /**
     * Configure the button bindings
     */
    private void configureButtonBindings() {
        // Reset robot pose and heading
        dResetSwerve.onTrue(swerve.resetGyro());
        dIntake.whileTrue(superstructure.intake());
        dShoot.whileTrue(superstructure.shoot());
        dFeedthrough.whileTrue(superstructure.feedthrough());
        // dPass.whileTrue(superstructure.passCommand());

        // dInitClimb.onTrue(Commands.print("Pivotting to climb position").alongWith(pivot.retractCommand()));
        // dClimb.onTrue(Commands.print("Running climb sequence").alongWith(pivot.retractCommand()));

        // dHubShot.whileTrue(Commands.print("Shooting static shot from from HUB"));
        // dTowerShot.whileTrue(Commands.print("Shooting static shot from from TOWER"));
        // dLeftCornerShot.whileTrue(Commands.print("Shooting static shot from from the left corner"));
        // dRightCornerShot.whileTrue(Commands.print("Shooting static shot from from the right corner"));

        // driver.y().whileTrue(
        //     flywheel.setFlywheelVelocityCommand(
        //         SmartDashboard.getNumber("Flywheel Set Point", 0), 
        //         SmartDashboard.getNumber("Flywheel Set Point", 0)
        //     ));

        // driver.a().whileTrue(
        //     hood.setHoodCommand(
        //         SmartDashboard.getNumber("Hood Angle", 0),
        //         SmartDashboard.getNumber("Hood Angle", 0)
        //     ));

        // driver.leftBumper().whileTrue(
        //     turret.driveManualCommand(0.05, 0.05)
        // ).onFalse(
        //     turret.driveManualCommand(0, 0)
        // );

        // driver.rightBumper().whileTrue(
        //     turret.driveManualCommand(-0.05, -0.05)
        // ).onFalse(
        //     turret.driveManualCommand(0, 0)
        // );

        // driver.y().whileTrue(flywheel.setFlywheelCommand(() -> SmartDashboard.getNumber("Flywheel RPS", 0)));
        // // driver.y().whileTrue(flywheel.setFlywheelPercentCommand(0, 1));
        // driver.a().whileTrue(hood.setHoodCommand(() -> SmartDashboard.getNumber("Hood Rotations", 0)));
        // driver.x().whileTrue(indexer.setPercentOutputCommand(1.0));
        // driver.a().whileTrue(hood.setHoodCommand(6, 6));
    }

    public void periodic() {
        // Add any periodic loop code to run here
    }
}