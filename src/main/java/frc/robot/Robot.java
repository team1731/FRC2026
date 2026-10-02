package frc.robot;

import java.io.File;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.Optional;

import org.littletonrobotics.junction.*;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.commands.*;
import com.pathplanner.lib.pathfinding.*;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc6328.LocalADStarAK;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * Main robot program entry point used by WPILib.
 *
 * <p>This class owns startup configuration, logging setup, autonomous preloading, and the
 * standard robot mode lifecycle callbacks. Most subsystem wiring lives in {@link RobotContainer}
 * so this class can stay focused on mode transitions.
 */
public class Robot extends LoggedRobot {
  private final RobotContainer container;
  private final AutoLoader autoLoader;

  private Command autonomousCommand = null;
  
  private boolean isRedAlliance = false;
  private boolean autoHasRan = false;

  /** Initializes robot-wide services, pathfinding warmups, logging, and brownout protection. */
  public Robot() {
    initLogging();
    container = RobotContainer.kInstance;
    autoLoader = AutoLoader.kInstance;
    Pathfinding.setPathfinder(new LocalADStarAK());
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    RobotController.setBrownoutVoltage(RobotConstants.kBrownoutVoltage);
    isRedAlliance = isRedAlliance();
  }

  /**
   * Sets up advantage kit and shuffleboard logging
   */
  private void initLogging() {
    // Log metadata (for AdvantageScope)
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    try {
      Logger.recordMetadata("Hostname", InetAddress.getLocalHost().getHostName().replaceAll("\\.local$", ""));
    } catch (UnknownHostException e) {
      Logger.recordMetadata("Hostname", "Unknown");
    }

    if (RobotConstants.kPublishLogsToNetworkTables) {
      Logger.addDataReceiver(new NT4Publisher());
    }
    if (Robot.isReal() && RobotConstants.kLogToWPILog) {
      File usbDrive = new File("/U");
      if (usbDrive.exists() && usbDrive.isDirectory()) {
        Logger.addDataReceiver(new WPILOGWriter()); // "/U/logs"
      } else {
        DriverStation.reportWarning("No USB stick detected - WPILOG file logging disabled for this session", false);
      }
    }

    Logger.start();
    SmartDashboard.updateValues();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * Preloads the autonomous mode and resets robot odometry if necessary
   */
  private void autoPreload() {
    Command curAuto = autoLoader.getSelected();
    boolean currentlyRed = isRedAlliance();

    if ((!curAuto.equals(autonomousCommand) || isRedAlliance != currentlyRed) && !autoHasRan) {
      autoLoader.getStartPose().ifPresent(startPose -> {
        SwerveSubsystem.kInstance.resetPose(startPose);
        SwerveSubsystem.kInstance.handler.resetVSLAMPose(startPose);
        SwerveSubsystem.kInstance.handler.resetAprilTagSimPose(startPose);
      });
    }

    autonomousCommand = curAuto;
    isRedAlliance = currentlyRed;
  }

  /**
   * The alliance we are currently on
   */
  /**
   * Returns the alliance reported by Driver Station, or {@code null} when unavailable.
   *
   * @return current alliance, or {@code null} before Driver Station provides one
   */
  public static Alliance getAlliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get();
    }
    return null;
  }

  /**
   * Whether we are the red alliance
   */
  /**
   * Checks whether Driver Station currently reports red alliance.
   *
   * @return true when on red alliance, false when blue or unknown
   */
  public static boolean isRedAlliance() {
    return getAlliance() != null ? getAlliance().equals(Alliance.Red) : false;
  }

  /** Runs the command scheduler and robot container periodic hooks every loop. */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    container.periodic();
  }

  /** Called once when the robot becomes disabled. */
  @Override
  public void disabledInit() {}

  /** Keeps the selected autonomous routine and starting pose ready while disabled. */
  @Override
  public void disabledPeriodic() {
    autoPreload();
  }

  /** Called once when the robot leaves disabled mode. */
  @Override
  public void disabledExit() {}

  /** Schedules the selected autonomous command and applies autonomous current limits. */
  @Override
  public void autonomousInit() {
    autonomousCommand = autoLoader.getSelected();
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }

    SwerveSubsystem.kInstance.setStatorCurrentLimit(SwerveConstants.kAutoCurrentLimit);
    autoHasRan = true;
  }

  /** Called every loop during autonomous after the scheduler has run. */
  @Override
  public void autonomousPeriodic() {}

  /** Called once when autonomous mode exits. */
  @Override
  public void autonomousExit() {}

  /** Cancels any remaining autonomous command and restores teleop current limits. */
  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    SwerveSubsystem.kInstance.setStatorCurrentLimit(SwerveConstants.kTeleCurrentLimit);
  }

  /** Called every loop during teleop after the scheduler has run. */
  @Override
  public void teleopPeriodic() {}

  /** Called once when teleop mode exits. */
  @Override
  public void teleopExit() {}

  /** Cancels all commands and restores safe teleop current limits for test mode. */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    SwerveSubsystem.kInstance.setStatorCurrentLimit(SwerveConstants.kTeleCurrentLimit);
  }

  /** Called every loop during test mode after the scheduler has run. */
  @Override
  public void testPeriodic() {}

  /** Called once when test mode exits. */
  @Override
  public void testExit() {}
}
