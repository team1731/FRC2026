package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj2.command.*;
import frc.lib.frc1678.mechviz.RobotVisualizer;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intakedeploy.IntakeDeploySubsystem;
import frc.robot.subsystems.intakeroller.IntakeRollerSubsystem;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * Builds the robot's subsystem graph and binds controllers to commands.
 *
 * <p>This is the main composition root for command-based robot code. Hardware should be owned by
 * subsystems, while this class decides which commands are defaults and which controls schedule them.
 */
public class RobotContainer {
  private static final Controls controls = new Controls();
  
  /** Shared container instance used by {@link Robot}. */
  public static final RobotContainer kInstance = new RobotContainer();

  // Subsystems
  public static Superstructure superstructure;
  public static SwerveSubsystem swerve;
  public static KickerSubsystem kicker;
  public static IndexerSubsystem indexer;
  public static HopperSubsystem hopper;
  public static HoodSubsystem hood;
  public static FlywheelSubsystem flywheel;
  public static IntakeRollerSubsystem intake;
  public static IntakeDeploySubsystem deploy;
  public static RobotVisualizer visualizer;

  

  private RobotContainer() {
    superstructure = Superstructure.kInstance;
    swerve = SwerveSubsystem.kInstance;
    kicker = KickerSubsystem.kInstance;
    indexer = IndexerSubsystem.kInstance;
    hopper = HopperSubsystem.kInstance;
    hood = HoodSubsystem.kInstance;
    flywheel = FlywheelSubsystem.kInstance;
    intake = IntakeRollerSubsystem.kInstance;
    deploy = IntakeDeploySubsystem.kInstance;

    new EventTrigger("Shoot").onTrue(superstructure.shoot(false));
    new EventTrigger("StopShoot").onTrue(superstructure.stopShoot());
    new EventTrigger("Intake").whileTrue(superstructure.intake());
    new EventTrigger("Warmup").whileTrue(superstructure.warmupWithHood());
    NamedCommands.registerCommand("TargetLock", superstructure.lockSwerveToHub());

    // Use RobotVisualizer.none() here for a template with no visualization work/output.
    visualizer = RobotState.createMechViz();
    visualizer.init();
    configureBindings();
  }

  /**
   * Maps driver and operator inputs to command actions.
   */
  private void configureBindings() {
    controls.resetSwerve
      .onTrue(new InstantCommand(() -> swerve.seedFieldCentric()));

    controls.intake
      .whileTrue(superstructure.intake());

    controls.shoot
      .whileTrue(superstructure.shoot(false))
      .onFalse(hopper.collapse());

    controls.shotFeedthrough
      .whileTrue(superstructure.shoot(true))
      .onFalse(hopper.collapse());

    controls.pass
      .whileTrue(superstructure.pass(false))
      .onFalse(hopper.collapse());

    controls.passFeedthrough
      .whileTrue(superstructure.pass(true))
      .onFalse(hopper.collapse());

    controls.overrideHubShot
      .whileTrue(superstructure.hubShot())
      .onFalse(hopper.collapse());

    controls.overrideTrenchShot
      .whileTrue(superstructure.trenchShot())
      .onFalse(hopper.collapse());

    controls.overrideTowerShot
      .whileTrue(superstructure.towerShot())
      .onFalse(hopper.collapse());
    
    controls.warmup
      .whileTrue(superstructure.warmupWithoutHood());

    // controls.raiseHopper
    //   .onTrue(hopper.extend());

    // controls.lowerHopper
    //   .onTrue(hopper.collapse());
  }

  /**
   * Runs container-owned periodic work from {@link Robot#robotPeriodic()}.
   */
  public void periodic() {
    visualizer.loop();
  }
}