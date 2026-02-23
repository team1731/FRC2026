package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.feeder.IndexerSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;

public class RobotContainer {
    /* Subsystems */
    protected static SwerveSubsystem swerve;
    protected static LEDSubsystem led;
    protected static FlywheelSubsystem flywheel;
    protected static HoodSubsystem hood;
    protected static TurretSubsystem turret;
    protected static IndexerSubsystem indexer;
    protected static IntakeRollerSubsystem intake;
    protected static IntakePivotSubsystem pivot;

    protected static Superstructure superstructure;

    /* Driver Buttons */
    private final CommandXboxController driver = new CommandXboxController(0);
    private final Trigger dResetSwerve = driver.povRight();
    private final Trigger dShoot = driver.rightTrigger();
    private final Trigger dPass = driver.rightBumper();
    private final Trigger dIntake = driver.leftTrigger();
    private final Trigger dRetractIntake = driver.leftBumper();
    private final Trigger dInitClimb = driver.back();
    private final Trigger dClimb = driver.start();

    private final Trigger dHubShot = driver.a();
    private final Trigger dTowerShot = driver.y();
    private final Trigger dLeftCornerShot = driver.x();
    private final Trigger dRightCornerShot = driver.b();

    private final CommandXboxController testController = new CommandXboxController(2);

    private final Trigger dUnjam = new Trigger(() -> false); // TODO - Add unjam button

    /* Operator Buttons */

    public RobotContainer() {
        configureSubsystems();
        configureNamedCommands();
        // configureButtonBindings();
        configureTestButtons();
    }

    /**
     * Configure all active subsystems on the robot and set default commands
     */
    private void configureSubsystems() {
        swerve = new SwerveSubsystem(true);
        led = new LEDSubsystem(true);
        flywheel = new FlywheelSubsystem(true);
        hood = new HoodSubsystem(true);
        turret = new TurretSubsystem(true);
        indexer = new IndexerSubsystem(true);
        pivot = new IntakePivotSubsystem(true);
        intake = new IntakeRollerSubsystem(true);

        superstructure = new Superstructure(swerve, flywheel, hood, turret);

        // Drivetrain will execute this command periodically 
        // if no other command is active on the drivetrain
        swerve.setDefaultCommand(swerve.driveCommand(driver, () -> true));
        flywheel.setDefaultCommand(flywheel.stopCommand());
        intake.setDefaultCommand(intake.stopCommand());
        indexer.setDefaultCommand(indexer.stopCommand());
        // turret.setDefaultCommand(turret.setManualCommand(driver.getLeftX() / 10d));
        // hood.setDefaultCommand(hood.setManualCommand(driver.getRightY() / 10d));
        // led.setDefaultCommand(led.setFireCommand());
    }

    private void configureNamedCommands() {
        // Named commands useful for PathPlanner events
        // ex. NamedCommands.registerCommand("Example", new ExampleCommand());
    }

    /**
     * Configure the button bindings
     */
    private void configureButtonBindings() {
        // Reset robot pose and heading
        dResetSwerve.onTrue(new InstantCommand(() -> {
            Pose2d resetPosition = Robot.isRedAlliance() ? new Pose2d(10.38, 3.01, new Rotation2d(Math.toRadians(0)))
                : new Pose2d(7.168, 5.006, new Rotation2d(Math.toRadians(180)));
            swerve.resetPose(resetPosition);
        }));

        // dShoot.whileTrue(superstructure.shootFuelCommand());
        dPass.whileTrue(superstructure.passFuelCommand());
        dIntake.whileTrue(intake.setPercentOutputCommand(1.0).alongWith(pivot.deployCommand())).onFalse(intake.stopCommand());
        dRetractIntake.onTrue(pivot.retractCommand());
        dUnjam.whileTrue(pivot.unjamCommand());

        dInitClimb.onTrue(Commands.print("Pivotting to climb position"));
        dClimb.onTrue(Commands.print("Running climb sequence"));

        dHubShot.whileTrue(Commands.print("Shooting static shot from from HUB"));
        dTowerShot.whileTrue(Commands.print("Shooting static shot from from TOWER"));
        dLeftCornerShot.whileTrue(Commands.print("Shooting static shot from from the left corner"));
        dRightCornerShot.whileTrue(Commands.print("Shooting static shot from from the right corner"));

        // // dShoot.whileTrue(flywheel.setVelocityCommand(RotationsPerSecond.of(50)));
        // dShoot.whileTrue(flywheel.setPercentOutputCommand(0.8d).alongWith(feeder.setPercentOutputCommand(1.0)));
        // // driver.rightBumper().whileTrue(flywheel.tuneShotCommand());
        // driver.leftTrigger().whileTrue(roller.setPercentOutputCommand(1.0));
        // driver.povDown().whileTrue(pivot.setManualCommand(-0.1));
        // driver.povUp().whileTrue(pivot.setManualCommand(0.1));
        // driver.leftBumper().whileTrue(roller.setPercentOutputCommand(-1.0));
    }

    private void configureTestButtons() {
        // Reset robot pose and heading
        dResetSwerve.onTrue(new InstantCommand(() -> {
            Pose2d resetPosition = Robot.isRedAlliance() ? new Pose2d(10.38, 3.01, new Rotation2d(Math.toRadians(0)))
                : new Pose2d(7.168, 5.006, new Rotation2d(Math.toRadians(180)));
            swerve.resetPose(resetPosition);
        }));

        dShoot.whileTrue(flywheel.setManualCommand(0.5).alongWith(indexer.setPercentOutputCommand(1)));
        dIntake.whileTrue(intake.setPercentOutputCommand(1));
        driver.leftBumper().whileTrue(turret.driveManualCommand(-0.1, -0.1));
        driver.leftBumper().whileTrue(turret.driveManualCommand(0.1, 0.1));

        driver.y().whileTrue(hood.setManualCommand(0.1, 0.1));
        driver.a().whileTrue(hood.setManualCommand(-0.1, -0.1));

        driver.b().whileTrue(pivot.setManualCommand(0.1));
        driver.x().whileTrue(pivot.setManualCommand(-0.1));
    }

    public void periodic() {
        // Add any periodic loop code to run here
    }
}