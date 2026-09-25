package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj2.command.Commands;
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
    private final CommandPS5Controller driver = new CommandPS5Controller(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    private final Trigger oHubShot = operator.a();
    private final Trigger oTowerShot = operator.x();
    private final Trigger oBumpShot = operator.y();
    private final Trigger oTunedShot = operator.b();
    private final Trigger oWarmup = operator.rightTrigger();

    private final Trigger oRaiseSqueezer = operator.rightBumper();
    private final Trigger oLowerSqueezer = operator.leftBumper();

    private final Trigger shotOverride = oHubShot.or(oTowerShot).or(oBumpShot).or(oTunedShot);

    private final Trigger dResetSwerve = driver.options();

    private final Trigger dIntake = driver.L2();
    private final Trigger dShoot = driver.R2();
    private final Trigger dPass = driver.R1();

    private final Trigger dFeedthrough = dIntake.and(dShoot);
    private final Trigger dPassthrough = dIntake.and(dPass);

    private final Trigger dRetract = driver.povUp();

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
        new EventTrigger("StopShoot").onTrue(superstructure.stopShooters());
        new EventTrigger("Intake").whileTrue(superstructure.intake());
        new EventTrigger("Warmup").whileTrue(superstructure.warmup());
        new EventTrigger("RaiseSqueezer").whileTrue(Commands.none());
        new EventTrigger("LowerSqueezer").whileTrue(Commands.none());
        NamedCommands.registerCommand("TargetLock", superstructure.lockSwerveToHub());
    }

    /**
     * Configure the button bindings
     */
    private void configureButtonBindings() {
        // Reset robot pose and heading
        dResetSwerve
            .onTrue(superstructure.resetSwerve());

        oWarmup
            .whileTrue(flywheel.setVelocity(40).alongWith(superstructure.stageFuel()));

        dIntake
            .and(() -> !dShoot.getAsBoolean() && !dPass.getAsBoolean())
            .whileTrue(
                superstructure.intake()
            );

        dShoot
            .and(shotOverride.negate())
            .whileTrue(superstructure.shoot())
            .onFalse(
                swerve.setLockingEnabled(false)
                    .alongWith(squeezer.squeeze())
            );

        oHubShot
            .and(dShoot)
            .and(oTunedShot.negate())
            .whileTrue(superstructure.defaultShot(60, 3))
            .onFalse(
                swerve.setLockingEnabled(false)
                    .alongWith(squeezer.squeeze())
            );

        oBumpShot
            .and(dShoot)
            .and(oTunedShot.negate())
            .whileTrue(superstructure.defaultShot(3.2))
            .onFalse(
                swerve.setLockingEnabled(false)
                    .alongWith(squeezer.squeeze())
            );

        oTowerShot
            .and(dShoot)
            .and(oTunedShot.negate())
            .whileTrue(superstructure.defaultShot(4.0))
            .onFalse(
                swerve.setLockingEnabled(false)
                    .alongWith(squeezer.squeeze())
            );

        oTunedShot
            .and(dShoot)
            .whileTrue(
                swerve.setLockingEnabled(false)
                    .andThen(superstructure.tunedShot())
            );

        dPass
            .and(oTunedShot.negate())
            .whileTrue(superstructure.pass())
            .onFalse(
                swerve.setLockingEnabled(false)
                    .alongWith(squeezer.squeeze())
            );

        dFeedthrough
            .and(oTunedShot.negate())
            .whileTrue(superstructure.feedthrough())
            .onFalse(swerve.setLockingEnabled(false));

        dPassthrough
            .and(oTunedShot.negate())
            .whileTrue(superstructure.passFeedthrough())
            .onFalse(swerve.setLockingEnabled(false));

        dRetract
            .whileTrue(
                pivot.retract()
                    .alongWith(squeezer.squeeze())
            );

        oRaiseSqueezer
            .onTrue(squeezer.raise());

        oLowerSqueezer
            .onTrue(squeezer.squeeze());
    }

    public void configureDefaultCommands() {
        // Drivetrain will execute this command periodically 
        // if no other command is active on the drivetrain
        swerve.setDefaultCommand(swerve.driveCommand(driver, () -> true));

        intake.setDefaultCommand(intake.stop());
        indexer.setDefaultCommand(indexer.stop());
        kicker.setDefaultCommand(kicker.stop());

        hood.setDefaultCommand(hood.stow());
        flywheel.setDefaultCommand(flywheel.warmup());
    }

    public void periodic() {
        // Add any periodic loop code to run here
    }
}
