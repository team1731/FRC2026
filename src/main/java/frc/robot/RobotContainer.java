package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj2.command.*;
import frc.lib.frc1678.mechviz.RobotVisualizer;
import frc.robot.subsystems.Superstructure;

/**
 * Builds the robot's subsystem graph and binds controllers to commands.
 *
 * <p>This is the main composition root for command-based robot code. Hardware should be owned by
 * subsystems, while this class decides which commands are defaults and which controls schedule them.
 */
public class RobotContainer {
  private static final Controls controls = new Controls();
  private Superstructure superstructure;
  private RobotVisualizer visualizer;

  public RobotContainer() {
    superstructure = new Superstructure();
    visualizer = RobotState.createMechViz();
    visualizer.init();

    new EventTrigger("Shoot").onTrue(superstructure.shoot(false));
    new EventTrigger("StopShoot").onTrue(superstructure.stopShoot());
    new EventTrigger("Intake").whileTrue(superstructure.intake());
    new EventTrigger("Warmup").whileTrue(Robot.flywheel.warmup());
    new EventTrigger("LowerSqueeze").onTrue(Robot.hopper.collapse());
    new EventTrigger("RaiseSqueeze").onTrue(Robot.hopper.extend());
    NamedCommands.registerCommand("TargetLock", superstructure.lockSwerveToHub());
    configureBindings();
  }

  /**
   * Maps driver and operator inputs to command actions.
   */
  private void configureBindings() {
    controls.resetSwerve
      .onTrue(new InstantCommand(() -> Robot.swerve.seedFieldCentric()));

    controls.intake
      .whileTrue(superstructure.intake());

    controls.shoot
      .whileTrue(superstructure.shoot(false))
      .onFalse(Robot.hopper.collapse());

    controls.shotFeedthrough
      .whileTrue(superstructure.shoot(true))
      .onFalse(Robot.hopper.collapse());

    controls.pass
      .whileTrue(superstructure.pass(false))
      .onFalse(Robot.hopper.collapse());

    controls.passFeedthrough
      .whileTrue(superstructure.pass(true))
      .onFalse(Robot.hopper.collapse());

    controls.overrideHubShot
      .whileTrue(superstructure.hubShot())
      .onFalse(Robot.hopper.collapse());

    controls.overrideTrenchShot
      .whileTrue(superstructure.trenchShot())
      .onFalse(Robot.hopper.collapse());

    controls.overrideTowerShot
      .whileTrue(superstructure.towerShot())
      .onFalse(Robot.hopper.collapse());
    
    controls.warmup
      .whileTrue(superstructure.warmupWithoutHood());

    controls.collapseIntake
      .onTrue(Robot.intakedeploy.home()
        .alongWith(Robot.hopper.collapse())
      );

    // controls.raiseHopper
    //   .onTrue(Robot.hopper.extend());

    // controls.lowerHopper
    //   .onTrue(Robot.hopper.collapse());
  }

  public void periodic() {
    superstructure.periodic();
    visualizer.loop();
  }
}