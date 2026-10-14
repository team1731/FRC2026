package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

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
  private Superstructure superstructure;
  private RobotVisualizer visualizer;

  public RobotContainer() {
    superstructure = new Superstructure();
    visualizer = RobotState.createMechViz();
    visualizer.init();
    Controls.build();

    NamedCommands.registerCommand("StopShoot", superstructure.stopShoot());
    NamedCommands.registerCommand("Intake", superstructure.intake());
    NamedCommands.registerCommand("Warmup", superstructure.warmupWithHood());
    NamedCommands.registerCommand("Shoot", superstructure.autoShoot().withTimeout(3.0));
    NamedCommands.registerCommand("TargetLock", superstructure.lockSwerveToHub());

    Controls.resetSwerve
      .onTrue(new InstantCommand(() -> Robot.swerve.resetTelePose()));

    Controls.snailDrive
      .whileTrue(new InstantCommand(() -> Robot.swerve.setSnailMode(true)))
      .onFalse(new InstantCommand(() -> Robot.swerve.setSnailMode(false)));

    Controls.intake
      .and((Controls.shoot.or(Controls.pass)).negate())
      .whileTrue(superstructure.intake());

    Controls.shoot
      .and((Controls.overrideHubShot
          .or(Controls.overrideTrenchShot)
          .or(Controls.overrideTowerShot))
          .negate())
      .and(Controls.intake.negate())
      .whileTrue(superstructure.shoot(false));

    Controls.shoot
      .and((Controls.overrideHubShot
          .or(Controls.overrideTrenchShot)
          .or(Controls.overrideTowerShot))
          .negate())
      .and(Controls.intake)
      .whileTrue(superstructure.shoot(true));

    Controls.pass
      .and(Controls.shoot.negate())
      .and(Controls.intake.negate())
      .whileTrue(superstructure.pass(false));

    Controls.pass
      .and(Controls.shoot.negate())
      .and(Controls.intake)
      .whileTrue(superstructure.pass(true));

    // Overrides require shoot to be held. If several are held, prefer hub, then trench, then tower.
    Controls.shoot
      .and(Controls.overrideHubShot)
      .whileTrue(superstructure.hubShot());

    Controls.shoot
      .and(Controls.overrideHubShot.negate())
      .and(Controls.overrideTrenchShot)
      .whileTrue(superstructure.trenchShot());

    Controls.shoot
      .and(Controls.overrideHubShot.negate())
      .and(Controls.overrideTrenchShot.negate())
      .and(Controls.overrideTowerShot)
      .whileTrue(superstructure.towerShot());
    
    Controls.warmup
      .and((Controls.shoot.or(Controls.pass)).negate())
      .whileTrue(superstructure.warmupWithoutHood());

    Controls.collapseIntake
      .onTrue(
        Robot.intakedeploy.home()
          .alongWith(Robot.hopper.collapse()));

    Controls.raiseHopper.onTrue(Robot.hopper.extend());

    Controls.lowerHopper.onTrue(Robot.hopper.collapse());
  }

  public void periodic() {
    superstructure.periodic();
    visualizer.loop();
  }
}