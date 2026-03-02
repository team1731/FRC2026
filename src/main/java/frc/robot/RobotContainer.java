package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.GoalTrackingCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.feeder.IndexerSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
//import frc.robot.subsystems.shooter.turret.TurretSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystemAI;
import frc.robot.subsystems.vision.limelight.AprilTagSubsystem;
import frc.robot.subsystems.vision.questnav.QuestNavSubsystem;

public class RobotContainer {
    /* Subsystems */
    protected static QuestNavSubsystem vslam;
    protected static AprilTagSubsystem aprilTag;

    protected static SwerveSubsystem swerve;
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
    private final Trigger dPass = driver.rightBumper();
    private final Trigger dIntake = driver.leftTrigger();
    private final Trigger dFeedthrough = driver.leftTrigger().and(driver.rightTrigger());
    private final Trigger dInitClimb = driver.back();
    private final Trigger dClimb = driver.start();

    private final Trigger dHubShot = driver.a();
    private final Trigger dTowerShot = driver.y();
    private final Trigger dLeftCornerShot = driver.x();
    private final Trigger dRightCornerShot = driver.b();

    private final Trigger dUnjam = new Trigger(() -> false); // TODO - Add unjam button

    // private static final LoggedTunableNumber flywheelSetpoint = new LoggedTunableNumber("FlywheelSetpoint", () -> true);
    // private static final LoggedTunableNumber hoodSetpoint = new LoggedTunableNumber("HoodAngle", () -> true);

    /* Operator Buttons */

    public RobotContainer() {
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
        swerve = new SwerveSubsystem(true);
        // led = new LEDSubsystem(true);
        flywheel = new FlywheelSubsystem(true);
        hood = new HoodSubsystem(true);
    //    turret = new TurretSubsystem(true);
        indexer = new IndexerSubsystem(true);
        pivot = new IntakePivotSubsystem(true);
        intake = new IntakeRollerSubsystem(true);


    // Left Turret: 200° CCW (+), -180° CW (-)
    m_leftTurret = new TurretSubsystemAI(
            "Left",
            22, // Motor CAN ID
            11, // CANcoder CAN ID
            0.0, // Offset (Rotations)
            -180.0, // Reverse Limit
            200.0, // Forward Limit
            39.111, (39.111 / 28.44), // rotor to sensor and sensor to mechanism
            // Velocity supplier for shooting on the move
            new Translation2d(0.10, 0.15),  //location of turret in meters
            swerve::getCurrentPose, // Pose Supplier
            swerve::getFieldRelativeChassisSpeeds);

    // Right Turret: 180° CCW (+), -200° CW (-) [Mirrored]
    m_rightTurret = new TurretSubsystemAI(
            "Right",
            18,
            12,
            0.0,
            -200.0, // Reverse Limit (further CW)
            180.0, // Forward Limit (shorter CCW)
            39.111, (39.111 / 28.44),
            new Translation2d(0.10, -0.15),
            swerve::getCurrentPose, // Same Pose Supplier
            swerve::getFieldRelativeChassisSpeeds

    );
    




        superstructure = new Superstructure(swerve, flywheel, hood, indexer, pivot, intake);
         vslam = new QuestNavSubsystem(true, swerve);
        aprilTag = new AprilTagSubsystem(true);

        // Drivetrain will execute this command periodically 
        // if no other command is active on the drivetrain
        swerve.setDefaultCommand(swerve.driveCommand(driver, () -> true));

         vslam.setDefaultCommand(() -> {
             vslam.addVisionMeasurement((pose, timestamp, stdDevs) -> {
                 swerve.addVisionMeasurement(pose, timestamp, stdDevs);
             });
         });



         aprilTag.setDefaultCommand(aprilTag.run(() -> {
             swerve.updateVisionOdometry();
             }).ignoringDisable(true)
             
         );

        flywheel.setDefaultCommand(flywheel.stopCommand());
        intake.setDefaultCommand(intake.stopCommand());
        indexer.setDefaultCommand(indexer.stopCommand());

        // hood.setDefaultCommand(hood.driveManualCommand(0, 0));
        hood.setDefaultCommand(hood.setHoodCommand(0, 0));
        // turret.setDefaultCommand(turret.setRightTurretCommand(() -> -swerve.getYaw() % 360d));
    }

    private void configureNamedCommands() {
        // Named commands useful for PathPlanner events
        // ex. NamedCommands.registerCommand("Example", new ExampleCommand());
        NamedCommands.registerCommand("Shoot", superstructure.shootCommand());
        NamedCommands.registerCommand("Intake", superstructure.intakeCommand());
        NamedCommands.registerCommand("Feedthrough", superstructure.feedthroughCommand());
        NamedCommands.registerCommand("Pass", Commands.none());
        NamedCommands.registerCommand("Climb", Commands.none());
        NamedCommands.registerCommand("Stow Hood", hood.stowHoodCommand());
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

        dIntake.whileTrue(superstructure.intakeCommand());
        dShoot.whileTrue(superstructure.shootCommand()).onFalse(hood.stowHoodCommand());
        dFeedthrough.whileTrue(superstructure.feedthroughCommand());


    
        // 2. Bind both turrets to a single button (e.g., Left Trigger)
        // While held, both track the goal. When released, both return to 0 (Safe Return).
       dHubShot.whileTrue(
            new GoalTrackingCommand(m_leftTurret, m_rightTurret)
        );
    
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