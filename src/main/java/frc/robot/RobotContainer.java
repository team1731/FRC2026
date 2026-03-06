package frc.robot;

import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.feeder.IndexerSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystemAI;
import frc.robot.subsystems.vision.limelight.AprilTagSubsystem;


public class RobotContainer {
    /* Subsystems */

    // protected static AprilTagSubsystem aprilTag;

    protected SwerveSubsystem swerve;
    // protected static LEDSubsystem led;
    protected static FlywheelSubsystem flywheel;
    protected static HoodSubsystem hood;
    //  protected static TurretSubsystem turret;
    protected static IndexerSubsystem indexer;
    protected static IntakeRollerSubsystem intake;
    protected static IntakePivotSubsystem pivot;
    protected static TurretSubsystemAI m_leftTurret;
    protected static TurretSubsystemAI m_rightTurret;

    protected static Superstructure superstructure;

    /* Driver Buttons */
    private final CommandXboxController driver = new CommandXboxController(0);
    private final Trigger dResetSwerve = driver.povRight();
    private final Trigger dShoot = driver.rightTrigger();
    private final Trigger dPass = driver.y();
    private final Trigger dIntake = driver.leftTrigger();
    private final Trigger dFeedthrough = driver.leftTrigger().and(driver.rightTrigger());

    private final Trigger dTowerShot = driver.a();
    private final Trigger dTrenchShot = driver.b();

    /* Operator Buttons */

    public RobotContainer(SwerveSubsystem swerve) {
        this.swerve = swerve;
        configureSubsystems();
        configureNamedCommands();
        configureButtonBindings();

        SmartDashboard.putNumber("Flywheel RPS", 50);
        SmartDashboard.putNumber("Hood Rotations", 3);
    }

    /**
     * Configure all active subsystems on the robot and set default commands
     */
    private void configureSubsystems() {
        flywheel = new FlywheelSubsystem(true);
        hood = new HoodSubsystem(true);
        indexer = new IndexerSubsystem(true);
        pivot = new IntakePivotSubsystem(true);
        intake = new IntakeRollerSubsystem(true);


        // Left Turret: 200° CCW (+), -180° CW (-)
        m_leftTurret = new TurretSubsystemAI(
            "Left",
            22, // Motor CAN ID
            true,
            29, // CANcoder CAN ID
            0.0185546875, // Offset (Rotations)0.018798828125
            .282, //discontinuty
            -303.92, // Reverse Limit
            93.0, // Forward Limit
            35.555, 1.0/(35.555 / 28.44), // rotor to sensor and sensor to mechanism
            // Velocity supplier for shooting on the move
            kLeftTurretToRobot.getTranslation().toTranslation2d(),  //location of turret in meters
            swerve::getCurrentPose, // Pose Supplier
            "Right CANivore");

        // Right Turret: 180° CCW (+), -200° CW (-) [Mirrored]
        m_rightTurret = new TurretSubsystemAI(
            "Right",
            18,
            true,
            30,
            -0.19482421875,
            0.742, //discontinuity
           -92.5, // Reverse Limit (further CW)
            319.7, // Forward Limit (shorter CCW)
            35.555, 1.0/(35.555 / 28.44),
            kRightTurretToRobot.getTranslation().toTranslation2d(),
            swerve::getCurrentPose, // Same Pose Supplier
            "Left CANivore");

        superstructure = new Superstructure(swerve, flywheel, hood, 
                                                indexer, pivot, intake, 
                                                    m_leftTurret, m_rightTurret);

        // aprilTag = new AprilTagSubsystem(true);

        // Drivetrain will execute this command periodically 
        // if no other command is active on the drivetrain
        swerve.setDefaultCommand(swerve.driveCommand(driver, () -> true));

        flywheel.setDefaultCommand(flywheel.stopCommand());
        intake.setDefaultCommand(intake.stopCommand());
        indexer.setDefaultCommand(indexer.stopCommand());

        hood.setDefaultCommand(hood.stowHoodCommand());
        m_leftTurret.setDefaultCommand(m_leftTurret.trackHubCommand());
        m_rightTurret.setDefaultCommand(m_rightTurret.trackHubCommand());
    }

    private void configureNamedCommands() {
        // Named commands useful for PathPlanner events
        // ex. NamedCommands.registerCommand("Example", new ExampleCommand());
        NamedCommands.registerCommand("Shoot", superstructure.immediateShootCommand().withTimeout(3));
        NamedCommands.registerCommand("Intake", superstructure.intakeCommand());
        NamedCommands.registerCommand("Feedthrough", superstructure.feedthroughFuelCommand());
        NamedCommands.registerCommand("Pass", Commands.none());
        NamedCommands.registerCommand("Climb", Commands.none());
        NamedCommands.registerCommand("Stow Hood", hood.stowHoodCommand());
        NamedCommands.registerCommand("Warmup", superstructure.warmupCommand());
    }

    /**
     * Configure the button bindings
     */
    private void configureButtonBindings() {
        // Reset robot pose and heading
        dResetSwerve.onTrue(new InstantCommand(() -> {
            swerve.resetHeadingButtonPressed();
        }));

        dIntake.whileTrue(superstructure.intakeCommand());
        dShoot.whileTrue(superstructure.shootFuelCommand()).onFalse(hood.stowHoodCommand());
        dFeedthrough.whileTrue(superstructure.feedthroughFuelCommand());

        dPass.whileTrue(superstructure.passFuelCommand()).onFalse(hood.stowHoodCommand());

        dTowerShot.whileTrue(superstructure.shootFuelCommand(3));
        dTrenchShot.whileTrue(superstructure.shootFuelCommand(4));

        // driver.rightTrigger().whileTrue(indexer.setPercentOutputCommand(1));
    
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