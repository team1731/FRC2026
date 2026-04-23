package frc.robot;

import static frc.robot.subsystems.drive.SwerveConstants.kAutoCurrentLimit;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.lib.frc6328.LoggedTunableNumber;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.squeezer.SqueezerSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.kicker.KickerSubsystem;
public class RobotContainer {
    public enum TestShotCondition {
        kNone,
        kDistance,
        kParameters,
    }

    private static TestShotCondition testCondition = TestShotCondition.kParameters;

    /* Subsystems */
    private SwerveSubsystem swerve;
    private IndexerSubsystem indexer;
    private KickerSubsystem kicker;
    private IntakeRollerSubsystem intake;
    private IntakePivotSubsystem pivot;
    private FlywheelSubsystem flywheel;
    private HoodSubsystem hood;
    private SqueezerSubsystem squeezer;

    // private LEDSubsystem led;

    private Superstructure superstructure;

    /* Driver Buttons */
    private final CommandXboxController driver = new CommandXboxController(0);
    private final Trigger dResetSwerve = driver.start();

    private final Trigger dIntake = driver.leftTrigger();
    private final Trigger dShoot = driver.rightTrigger();
    private final Trigger dPass = driver.y();

    private final Trigger dFeedthrough = dIntake.and(dShoot);
    private final Trigger dPassthrough = dIntake.and(dPass);

    private final Trigger dTrenchShot = driver.b();
    private final Trigger dTowerShot = driver.a();
    private final Trigger dHubShot = driver.x();
    
    // private final Trigger dTestSetShot = driver.back();
    // private final Trigger dUnjam = driver.back();

    private final Trigger dSpit = driver.leftBumper();
    // private final Trigger dStationaryShot = driver.rightBumper();
    
    private final Trigger dRetract = driver.povUp();
    private final Trigger dRaiseCurrentLimit = driver.povLeft();

    private final Trigger dRaiseSqueezer = driver.rightBumper();

    public RobotContainer(SwerveSubsystem swerve) {
        this.swerve = swerve;
        configureSubsystems();
        configureButtonBindings();
        configureDefaultCommands();
        configureNamedCommands();

        SmartDashboard.putNumber("TuneableFlywheelRPS", 80.0);
        SmartDashboard.putNumber("TuneableHoodRotations", 0);
    }

    /**
     * Configure all active subsystems on the robot and set default commands
     */
    private void configureSubsystems() {
        flywheel = new FlywheelSubsystem(true);
        hood = new HoodSubsystem(true);
        indexer = new IndexerSubsystem(true);
        kicker = new KickerSubsystem(true);
        pivot = new IntakePivotSubsystem(true);
        intake = new IntakeRollerSubsystem(true);
        squeezer = new SqueezerSubsystem(false); // Disabled for now until we can confirm it is on the robot
        // led = new LEDSubsystem(true);

        superstructure = new Superstructure(swerve, flywheel, hood, indexer, kicker, pivot, intake);
    }

    private void configureNamedCommands() {
        // Named commands useful for PathPlanner events
        // ex. NamedCommands.registerCommand("Example", new ExampleCommand());
        // NamedCommands.registerCommand("SitAndShoot", superstructure.autoShoot());
        // NamedCommands.registerCommand("StowShooter", superstructure.stopShooters());
        // NamedCommands.registerCommand("StopIntakeCommand", intake.stop());
        // NamedCommands.registerCommand("IntakeCommand", superstructure.runIntake(true));
        // NamedCommands.registerCommand("PassCommand", superstructure.pass());
        // NamedCommands.registerCommand("WarmupCommand", superstructure.warmup());
        // NamedCommands.registerCommand("FeedthroughCommand", superstructure.feedthrough());

        // new EventTrigger("Shoot").onTrue(superstructure.shoot());
        // new EventTrigger("StopShoot").onTrue(superstructure.stopShooters());
        // new EventTrigger("Intake").whileTrue(superstructure.runIntake(true));
        // new EventTrigger("Pass").onTrue(superstructure.pass());
        // new EventTrigger("Warmup").whileTrue(superstructure.warmup());
        // new EventTrigger("Feedthrough").whileTrue(superstructure.feedthrough());
    }

    /**
     * Configure the button bindings
     */
    private void configureButtonBindings() {
        // Reset robot pose and heading
        dResetSwerve.onTrue(superstructure.resetSwerve());

        dIntake.and(() -> !dShoot.getAsBoolean() && !dPass.getAsBoolean()).whileTrue(superstructure.runIntake(true));
        dShoot.whileTrue(superstructure.shoot()).onFalse(swerve.setLockingEnabled(false));
        dPass.whileTrue(superstructure.pass()).onFalse(swerve.setLockingEnabled(false));
        dFeedthrough.whileTrue(superstructure.feedthrough()).onFalse(swerve.setLockingEnabled(false));
        dPassthrough.whileTrue(superstructure.passFeedthrough()).onFalse(swerve.setLockingEnabled(false));

        // dStationaryShot.whileTrue(superstructure.stationaryShot()).onFalse(swerve.setLockingEnabled(false));

        dHubShot.whileTrue(superstructure.defaultShot(100.0, 15)).onFalse(swerve.setLockingEnabled(false));
        dTowerShot.whileTrue(superstructure.defaultShot(80.0, 8)).onFalse(swerve.setLockingEnabled(false));
        dTrenchShot.whileTrue(superstructure.defaultShot(80.0, 14)).onFalse(swerve.setLockingEnabled(false));
        
        // (dTestSetShot.and(() -> testCondition.equals(TestShotCondition.kDistance)))
        //     .whileTrue(superstructure.tuneShot(tuneableDistanceShot, true));
        // (dTestSetShot.and(() -> testCondition.equals(TestShotCondition.kParameters)))
        //     .whileTrue(superstructure.tuneShot(tuneableFlywheelRPS.get(), tuneableHoodRotations.get(), true));

        // driver.back().whileTrue(superstructure.defaultShot(
        //     () -> SmartDashboard.getNumber("TuneableFlywheelRPS", 50.0), 
        //     () -> SmartDashboard.getNumber("TuneableHoodRotations", 0.0)
        // ));

        dSpit.whileTrue(superstructure.spit());

        dRaiseSqueezer.whileTrue(squeezer.raise());

        // driver.back().whileTrue(superstructure.tuneShot(tuneableDistanceShot, true));
        dRetract.whileTrue(pivot.retract());
        dRaiseCurrentLimit.onTrue(new InstantCommand(() -> swerve.setStatorCurrentLimit(kAutoCurrentLimit)));
    }

    public void configureDefaultCommands() {
        // Drivetrain will execute this command periodically 
        // if no other command is active on the drivetrain
        swerve.setDefaultCommand(swerve.driveCommand(driver, () -> true));

        intake.setDefaultCommand(intake.stop());
        indexer.setDefaultCommand(indexer.stop());
        kicker.setDefaultCommand(kicker.stop());

        hood.setDefaultCommand(hood.stow());
        flywheel.setDefaultCommand(flywheel.stop());

        // led.setDefaultCommand(led.flashAllianceShift());
    }

    public void periodic() {
        // Add any periodic loop code to run here
    }
}