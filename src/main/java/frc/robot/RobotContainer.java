package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Superstructure;

/**
 * Builds the robot's subsystem graph and binds controllers to commands.
 *
 * <p>This is the main composition root for command-based robot code. Hardware should be owned by
 * subsystems, while this class decides which commands are defaults and which controls schedule them.
 */
public class RobotContainer {
  private Superstructure superstructure;
  // private RobotVisualizer visualizer;

  public RobotContainer() {
    superstructure = new Superstructure();
    // visualizer = RobotState.createMechViz();
    // visualizer.init();
    Controls.build();

    // NamedCommands.registerCommand("RaiseSqueezer", Robot.hopper.extend());
    // NamedCommands.registerCommand("LowerSqueezer", Robot.hopper.collapse()); 

    Controls.resetSwerve
      .onTrue(new InstantCommand(() -> Robot.swerve.resetTelePose()));

    Controls.snailDrive
      .whileTrue(new InstantCommand(() -> Robot.swerve.setSnailMode(true)))
      .onFalse(new InstantCommand(() -> Robot.swerve.setSnailMode(false)));
  }

  public void periodic() {
    superstructure.periodic();
    // visualizer.loop();
  }
}