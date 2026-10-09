package frc.robot;

import java.io.File;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.Optional;

import org.littletonrobotics.junction.*;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import frc.lib.frc1731.EventLogWriter;

import com.pathplanner.lib.commands.*;
import com.pathplanner.lib.pathfinding.*;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.frc6328.LocalADStarAK;
import frc.robot.subsystems.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intakedeploy.IntakeDeploySubsystem;
import frc.robot.subsystems.intakeroller.IntakeRollerSubsystem;
import frc.robot.subsystems.kicker.KickerSubsystem;
import frc.robot.subsystems.power.PowerSubsystem;
import frc.robot.subsystems.swerve.*;

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
  
  public static SwerveSubsystem swerve;
  public static IntakeDeploySubsystem intakedeploy;
  public static IntakeRollerSubsystem intakeroller;
  public static IndexerSubsystem indexer;
  public static KickerSubsystem kicker;
  public static HoodSubsystem hood;
  public static FlywheelSubsystem flywheel;
  public static HopperSubsystem hopper;
  public static PowerSubsystem power;

  private Command autonomousCommand = null;
  private boolean isRedAlliance = false;
  private boolean autoHasRan = false;
  
  public static final Field2d field = new Field2d();

  /** Initializes robot-wide services, pathfinding warmups, logging, and brownout protection. */
  public Robot() {
    swerve = new SwerveSubsystem();
    intakedeploy = new IntakeDeploySubsystem();
    intakeroller = new IntakeRollerSubsystem();
    indexer = new IndexerSubsystem();
    kicker = new KickerSubsystem();
    hood = new HoodSubsystem();
    flywheel = new FlywheelSubsystem();
    hopper = new HopperSubsystem();
    power = new PowerSubsystem();
    
    power.addSimMotor("IntakeDeploy", intakedeploy.getMotor());
    power.addSimMotor("IntakeRoller", intakeroller.getMotor());
    power.addSimMotor("Indexer", indexer.getMotor());
    power.addSimMotor("Kicker", kicker.getMotor());
    power.addSimMotor("Hood", hood.getMotor());
    power.addSimMotor("Flywheel", flywheel.getMotor());
    power.addSimMotor("Hopper", hopper.getMotor());
    power.addSimLoad("Swerve", swerve::getSimSupplyCurrent);

    // Container and autoloader MUST be instantiated after subsystems
    container = new RobotContainer();
    autoLoader = new AutoLoader();

    initLogging();
    Pathfinding.setPathfinder(new LocalADStarAK());
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    RobotController.setBrownoutVoltage(RobotConstants.kBrownoutVoltage);
    isRedAlliance = isRedAlliance();

    SmartDashboard.putData("Field", field);
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

    if (RobotConstants.kPublishLogsToNT) {
      Logger.addDataReceiver(new NT4Publisher());
    }

    // External dashboards need a server even when AdvantageKit disables LiveWindow.
    if (isSimulation()) {
      NetworkTableInstance.getDefault().startServer("", "", 1735, 5810);
    }
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    
    if (Robot.isReal() && RobotConstants.kLogToWPILog) {
      File usbDrive = new File("/U");
      if (usbDrive.exists() && usbDrive.isDirectory()) {
        Logger.addDataReceiver(new EventLogWriter("/U/logs", RobotConstants.kLogEventKey));
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
        Robot.swerve.resetPose(startPose);
        Robot.swerve.handler.resetVSLAMPose(startPose);
        Robot.swerve.handler.resetAprilTagSimPose(startPose);
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
    GameState.logValues();
    container.periodic();
  }

  @Override
  public void simulationPeriodic() {
    power.updateSimulation();
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
  }

  /** Called every loop during test mode after the scheduler has run. */
  @Override
  public void testPeriodic() {}

  /** Called once when test mode exits. */
  @Override
  public void testExit() {}
}
