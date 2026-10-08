package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.frc1678.mechviz.RobotVisualizer;
import frc.robot.subsystems.Superstructure;

/**
 * Builds the robot's subsystem graph and binds controllers to commands.
 *
 * <p>This is the main composition root for command-based robot code. Hardware should be owned by
 * subsystems, while this class decides which commands are defaults and which controls schedule them.
 */
public class RobotContainer {
  private Superstructure superstructure;
  private RobotVisualizer visualizer;

  public RobotContainer() {
    superstructure = new Superstructure();
    visualizer = RobotState.createMechViz();
    visualizer.init();
    Controls.build();

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
    Controls.resetSwerve
      .onTrue(new InstantCommand(() -> Robot.swerve.resetTelePose()));

    Controls.snailDrive
      .whileTrue(new InstantCommand(() -> Robot.swerve.setSnailMode(true)))
      .onFalse(new InstantCommand(() -> Robot.swerve.setSnailMode(false)));

    // Shoot takes priority over pass; each mode has its own trigger expression.
    Trigger firing = Controls.shoot.or(Controls.pass);
    Trigger passing = Controls.pass.and(Controls.shoot.negate());
    Trigger overrideShot = Controls.overrideHubShot
      .or(Controls.overrideTrenchShot).or(Controls.overrideTowerShot);
    Trigger normalShot = Controls.shoot.and(overrideShot.negate());

    Controls.intake.and(firing.negate())
      .whileTrue(superstructure.intake());

    normalShot.and(Controls.intake.negate())
      .whileTrue(superstructure.shoot(false));
    normalShot.and(Controls.intake)
      .whileTrue(superstructure.shoot(true));

    passing.and(Controls.intake.negate())
      .whileTrue(superstructure.pass(false));
    passing.and(Controls.intake)
      .whileTrue(superstructure.pass(true));

    // Overrides require shoot to be held. If several are held, prefer hub, then trench, then tower.
    Controls.shoot.and(Controls.overrideHubShot)
      .whileTrue(superstructure.hubShot());
    Controls.shoot.and(Controls.overrideHubShot.negate()).and(Controls.overrideTrenchShot)
      .whileTrue(superstructure.trenchShot());
    Controls.shoot.and(Controls.overrideHubShot.negate())
      .and(Controls.overrideTrenchShot.negate()).and(Controls.overrideTowerShot)
      .whileTrue(superstructure.towerShot());

    // Do not collapse the hopper while transitioning between firing modes.
    firing.onFalse(Robot.hopper.collapse());
    
    // Controls.warmup
    //   .and(firing.negate())
    //   .whileTrue(superstructure.warmupWithoutHood());

    Controls.collapseIntake
      .onTrue(
        Robot.intakedeploy.home()
          .alongWith(Robot.hopper.collapse())
      );

    // Controls.raiseHopper
    //   .onTrue(Robot.hopper.extend());

    // Controls.lowerHopper
    //   .onTrue(Robot.hopper.collapse());
  }

  public void periodic() {
    superstructure.periodic();
    visualizer.loop();
  }
}
