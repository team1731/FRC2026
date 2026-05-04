package frc.robot;

import static frc.robot.subsystems.drive.SwerveConstants.kAutoCurrentLimit;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.squeezer.SqueezerSubsystem;
import frc.robot.subsystems.intake.pivot.IntakePivotSubsystem;
import frc.robot.subsystems.intake.roller.IntakeRollerSubsystem;
import frc.robot.subsystems.kicker.KickerSubsystem;

public class RobotContainer {
    private Superstructure superstructure;

    /* Subsystems */
    private SwerveSubsystem swerve;
    private IndexerSubsystem indexer;
    private KickerSubsystem kicker;
    private IntakeRollerSubsystem intake;
    private IntakePivotSubsystem pivot;
    private FlywheelSubsystem flywheel;
    private HoodSubsystem hood;
    private SqueezerSubsystem squeezer;

    /* Driver Buttons */
    // Unused buttons: back, pov down/right
    private final CommandXboxController driver = new CommandXboxController(0);
    private final Trigger dResetSwerve = driver.start();

    private final Trigger dIntake = driver.leftTrigger();
    private final Trigger dShoot = driver.rightTrigger();
    private final Trigger dPass = driver.y();

    private final Trigger dIntakeNoHopper = driver.leftBumper();

    private final Trigger dFeedthrough = dIntake.and(dShoot);
    private final Trigger dPassthrough = dIntake.and(dPass);

    private final Trigger dTrenchShot = driver.b();
    private final Trigger dManualPass = driver.a();
    private final Trigger dHubShot = driver.x();

    private final Trigger dRetract = driver.povUp();
    private final Trigger dRaiseCurrentLimit = driver.povLeft();

    public RobotContainer(SwerveSubsystem swerve) {
        this.swerve = swerve;
        configureSubsystems();
        configureButtonBindings();
        configureDefaultCommands();
        configureNamedCommands();
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
        squeezer = new SqueezerSubsystem(true);
        // led = new LEDSubsystem(true);

        superstructure = new Superstructure(swerve, flywheel, hood, indexer, kicker, pivot, intake, squeezer);
    }

    private void configureNamedCommands() {
        // Named commands useful for PathPlanner events
        // ex. NamedCommands.registerCommand("Example", new ExampleCommand());
        new EventTrigger("Shoot").onTrue(superstructure.autoShoot());
        // new EventTrigger("Feedthrough").whileTrue(superstructure.feedthrough());
        new EventTrigger("StopShoot").onTrue(superstructure.stopShooters());
        new EventTrigger("Intake").whileTrue(superstructure.runIntake(true));
        new EventTrigger("RaiseSqueezer").onTrue(squeezer.raise());
        new EventTrigger("LowerSqueezer").onTrue(squeezer.squeeze());
        new EventTrigger("Warmup").whileTrue(superstructure.warmup());
        // new EventTrigger("StopTrack").onTrue(swerve.stopLocking());
        NamedCommands.registerCommand("TargetLock", superstructure.lockSwerveToHub());
    }

    /**
     * Configure the button bindings
     */
    private void configureButtonBindings() {
        // Reset robot pose and heading
        dResetSwerve.onTrue(superstructure.resetSwerve());

        dIntake.and(() -> !dShoot.getAsBoolean() && !dPass.getAsBoolean()).whileTrue(superstructure.runIntake(true).alongWith(squeezer.raise()));
        dShoot.whileTrue(superstructure.shoot()).onFalse(swerve.setLockingEnabled(false));
        dPass.whileTrue(superstructure.pass()).onFalse(swerve.setLockingEnabled(false));
        dFeedthrough.whileTrue(superstructure.feedthrough()).onFalse(swerve.setLockingEnabled(false));
        dPassthrough.whileTrue(superstructure.passFeedthrough()).onFalse(swerve.setLockingEnabled(false));

        dIntakeNoHopper.and(() -> !dShoot.getAsBoolean() && !dPass.getAsBoolean()).whileTrue(superstructure.runIntake(true).alongWith(squeezer.squeeze()));

        dHubShot.whileTrue(superstructure.defaultShot(60, 3)).onFalse(swerve.setLockingEnabled(false));
        dManualPass.whileTrue(superstructure.defaultShot(75, 18)).onFalse(swerve.setLockingEnabled(false));
        dTrenchShot.whileTrue(superstructure.defaultShot(3.2)).onFalse(swerve.setLockingEnabled(false));
        
        // dSpit.whileTrue(superstructure.spit());

        dRetract.whileTrue(pivot.retract());
        dRaiseCurrentLimit.onTrue(new InstantCommand(() -> swerve.setStatorCurrentLimit(kAutoCurrentLimit)));
        // driver.povDown().onTrue(swerve.launchQuestnav());
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

        // squeezer.setDefaultCommand(squeezer.stop());

        // led.setDefaultCommand(led.flashAllianceShift());
    }

    public void periodic() {
        // Add any periodic loop code to run here
    }
}