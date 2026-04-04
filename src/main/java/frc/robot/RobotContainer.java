package frc.robot;

import static frc.robot.subsystems.shooter.flywheel.FlywheelConstants.*;
import static frc.robot.subsystems.shooter.hood.HoodConstants.*;
import static frc.robot.subsystems.shooter.turret.TurretConstants.*;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.lib.frc6328.LoggedTunableNumber;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.shooter.hood.HoodSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.shooter.turret.TurretSubsystem;
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
    private IntakeRollerSubsystem intake;
    private IntakePivotSubsystem pivot;

    private TurretSubsystem leftTurret, rightTurret;
    private FlywheelSubsystem leftFlywheel, rightFlywheel;
    private HoodSubsystem leftHood, rightHood;

    private LEDSubsystem led;

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
    
    private final Trigger dTestSetShot = driver.back();

    private final Trigger dSpit = driver.leftBumper();
    private final Trigger dStationaryShot = driver.rightBumper();

    private final Trigger dLeftTurretLeft = driver.povLeft();
    private final Trigger dLeftTurretRight = driver.povRight();
    private final Trigger dRightTurretLeft = driver.povDown();
    private final Trigger dRightTurretRight = driver.povUp();

    private LoggedTunableNumber tuneableFlywheelRPS = new LoggedTunableNumber("TunedFlywheelRPS", 0, () -> testCondition.equals(TestShotCondition.kParameters));
    private LoggedTunableNumber tuneableHoodRotations = new LoggedTunableNumber("TunedHoodRotations", 0, () -> testCondition.equals(TestShotCondition.kParameters));
    private LoggedTunableNumber tuneableDistanceShot = new LoggedTunableNumber("TunedDistanceShot", 1.8, () -> testCondition.equals(TestShotCondition.kDistance));

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
        leftFlywheel = new FlywheelSubsystem(kLeftFlywheelConfig, true);
        rightFlywheel = new FlywheelSubsystem(kRightFlywheelConfig, true);

        leftHood = new HoodSubsystem(kLeftHoodConfig, true);
        rightHood = new HoodSubsystem(kRightHoodConfig, true);

        leftTurret = new TurretSubsystem(kLeftTurretConfigs, () -> swerve.getCurrentPose(), true);
        rightTurret = new TurretSubsystem(kRightTurretConfigs, () -> swerve.getCurrentPose(), true);

        indexer = new IndexerSubsystem(true);
        pivot = new IntakePivotSubsystem(true);
        intake = new IntakeRollerSubsystem(true);

        led = new LEDSubsystem(true);

        superstructure = new Superstructure(swerve, leftFlywheel, rightFlywheel, 
                                                leftHood, rightHood, 
                                                indexer, pivot, intake, 
                                                    leftTurret, rightTurret);
    }

    private void configureNamedCommands() {
        // Named commands useful for PathPlanner events
        // ex. NamedCommands.registerCommand("Example", new ExampleCommand());
        NamedCommands.registerCommand("Shoot", superstructure.autoShoot());
        NamedCommands.registerCommand("StopShoot", superstructure.stopShooters());
        NamedCommands.registerCommand("Intake", superstructure.runIntake(true));
        NamedCommands.registerCommand("Pass", superstructure.pass());
        NamedCommands.registerCommand("StowHood", Commands.deadline(Commands.waitSeconds(0.1), superstructure.stowHoodsOnce()));
        NamedCommands.registerCommand("Warmup", superstructure.warmup());
        NamedCommands.registerCommand("Feedthrough", superstructure.feedthrough());
    }

    /**
     * Configure the button bindings
     */
    private void configureButtonBindings() {
        // Reset robot pose and heading
        dResetSwerve.onTrue(superstructure.resetYaw());

        dIntake.and(() -> !dShoot.getAsBoolean() && !dPass.getAsBoolean()).whileTrue(superstructure.runIntake(true));
        dShoot.whileTrue(superstructure.shoot());
        dPass.whileTrue(superstructure.pass());
        dFeedthrough.whileTrue(superstructure.feedthrough());
        dPassthrough.whileTrue(superstructure.passFeedthrough());

        dStationaryShot.whileTrue(superstructure.stationaryShot());

        dHubShot.whileTrue(superstructure.manualShot(1.8, true));
        dTowerShot.whileTrue(superstructure.manualShot(2.5, true));
        dTrenchShot.whileTrue(superstructure.manualShot(4, true));
        
        (dTestSetShot.and(() -> testCondition.equals(TestShotCondition.kDistance)))
            .whileTrue(superstructure.tuneShot(tuneableDistanceShot, true));
        (dTestSetShot.and(() -> testCondition.equals(TestShotCondition.kParameters)))
            .whileTrue(superstructure.tuneShot(tuneableFlywheelRPS.get(), tuneableHoodRotations.get(), true));

        dSpit.whileTrue(superstructure.spit());

        dLeftTurretLeft.whileTrue(leftTurret.setDegrees(-200));
        dLeftTurretRight.whileTrue(leftTurret.setDegrees(80));
        dRightTurretLeft.whileTrue(rightTurret.setDegrees(-80));
        dRightTurretRight.whileTrue(rightTurret.setDegrees(200));
    }

    public void configureDefaultCommands() {
        // Drivetrain will execute this command periodically 
        // if no other command is active on the drivetrain
        swerve.setDefaultCommand(swerve.driveCommand(driver, () -> true));

        intake.setDefaultCommand(intake.stop());
        indexer.setDefaultCommand(indexer.stop());

        leftTurret.setDefaultCommand(leftTurret.trackHub());
        rightTurret.setDefaultCommand(rightTurret.trackHub());

        leftHood.setDefaultCommand(leftHood.stow());
        rightHood.setDefaultCommand(rightHood.stow());

        leftFlywheel.setDefaultCommand(leftFlywheel.stop());
        rightFlywheel.setDefaultCommand(rightFlywheel.stop());

        led.setDefaultCommand(led.flashAllianceShift());
    }

    public void periodic() {
        // Add any periodic loop code to run here
    }
}