package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.lib.frc1731.hardware.MotorIOTalonFX;
import frc.lib.frc1731.hardware.motor.MotorIOTest;
import frc.lib.frc1731.hardware.motor.PortConfig;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;

public class RobotContainer {
    /* Subsystems */
    protected static SwerveSubsystem swerve;
    protected static LEDSubsystem led;

    /* Driver Buttons */
    private final CommandXboxController xboxController = new CommandXboxController(0);
    private final Trigger dResetSwerve = xboxController.povRight();
    private final Trigger dShoot = xboxController.rightTrigger();

    private MotorIOTest protoHood, protoFlywheel, protoTurret;

    /* Operator Buttons */

    public RobotContainer() {
        configureSubsystems();
        configureNamedCommands();
        configureButtonBindings();
    }

    /**
     * Configure all active subsystems on the robot and set default commands
     */
    private void configureSubsystems() {
        swerve = new SwerveSubsystem(true);
        led = new LEDSubsystem(true);

        protoHood = new MotorIOTest(new PortConfig(30), MotorIOTalonFX.class)
            .setSoftLimits(30d, 0d);

        protoFlywheel = new MotorIOTest(new PortConfig(31), MotorIOTalonFX.class);

        protoTurret = new MotorIOTest(new PortConfig(32), MotorIOTalonFX.class)
            .setSoftLimits(10d, -10d);

        SmartDashboard.putNumber("Hood Angle Degrees", protoHood.getRotations() * 15d / 2432d * 360d + 20d);

        // Drivetrain will execute this command periodically 
        // if no other command is active on the drivetrain
        swerve.setDefaultCommand(swerve.drive(xboxController, () -> true));

        protoHood.setDefaultCommand(protoHood.run(() -> {protoHood.setPercentOutput(xboxController.getLeftY() / 4d);}));
        protoTurret.setDefaultCommand(protoTurret.run(() -> {protoHood.setPercentOutput(xboxController.getRightX() / 4d);}));
    }

    private void configureNamedCommands() {
        // Named commands useful for PathPlanner events
        // ex. NamedCommands.registerCommand("Example", new ExampleCommand());
    }

    /**
     * Configure the button bindings
     */
    private void configureButtonBindings() {
        // Reset robot pose and heading
        dResetSwerve.onTrue(new InstantCommand(() -> {
            Pose2d resetPosition = Robot.isRedAlliance() ? new Pose2d(10.38, 3.01, new Rotation2d(Math.toRadians(0)))
                : new Pose2d(7.168, 5.006, new Rotation2d(Math.toRadians(180)));
            swerve.resetPose(resetPosition);
        }));

        dShoot.whileTrue(protoFlywheel.setTuneablePercentOutput(0.8)).onFalse(protoFlywheel.setPercentOutput(0d));
    }

    /**
     * Called once when teleop is initialized
     */
    public void teleopInit() {

    }
}