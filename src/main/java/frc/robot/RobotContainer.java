package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.turret.TurretSubsystem;

public class RobotContainer {
    /* Subsystems */
    protected static SwerveSubsystem swerve;
    protected static LEDSubsystem led;
    protected static FlywheelSubsystem flywheel;
    protected static HoodSubsystem hood;
    protected static TurretSubsystem turret;

    protected static Superstructure superstructure;

    /* Driver Buttons */
    private final CommandXboxController driver = new CommandXboxController(0);
    private final Trigger dResetSwerve = driver.povRight();
    private final Trigger dShoot = driver.rightTrigger();

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
        led = new LEDSubsystem(true);
        flywheel = new FlywheelSubsystem(true);
        hood = new HoodSubsystem(true);
        turret = new TurretSubsystem(true);

        superstructure = new Superstructure(swerve, flywheel, hood, null);

        // Drivetrain will execute this command periodically 
        // if no other command is active on the drivetrain
        swerve.setDefaultCommand(swerve.driveCommand(driver, () -> true));
        flywheel.setDefaultCommand(flywheel.stopCommand());
        turret.setDefaultCommand(turret.setManualCommand(driver.getLeftX() / 2d));
        // hood.setDefaultCommand(hood.setManualCommand(driver.getRightY() / 2d));
        led.setDefaultCommand(led.setFireCommand());
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

        dShoot.whileTrue(flywheel.setVelocityCommand(RotationsPerSecond.of(50)));
        driver.rightBumper().whileTrue(flywheel.tuneShotCommand());

        driver.y().onTrue(hood.setAngleCommand(HoodConstants.kMaxAngle));
        driver.b().onTrue(hood.setAngleCommand(Degrees.of(20)));
        driver.a().onTrue(hood.setAngleCommand(HoodConstants.kStartAngle));
    }

    /**
     * Called once when teleop is initialized
     */
    public void teleopInit() {

    }
}