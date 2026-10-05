package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.subsystems.swerve.generated.TunerConstants;

/**
 * Reusable CTRE swerve request objects used by teleop, autonomous, and auto-alignment commands.
 */
public class SwerveRequests {
    private static final double kDriveToTargetMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private static final double kDriveToTargetDeadband = 0.01; // Add a 1% deadband

    /** Request that points all modules into a braking X pattern. */
    public static final SwerveRequest.SwerveDriveBrake kBrakeRequest = new SwerveRequest.SwerveDriveBrake();

    /** Request used by PathPlanner to apply robot-relative chassis speeds during auto. */
    public static final SwerveRequest.ApplyRobotSpeeds kAutoRequest = new SwerveRequest.ApplyRobotSpeeds();

    /** Field-centric facing-angle request used for closed-loop auto alignment. */
    public static final SwerveRequest.FieldCentricFacingAngle kAutoAlignRequest = new SwerveRequest.FieldCentricFacingAngle()
        .withRotationalDeadband(SwerveConstants.kMaxAngularRate * kDriveToTargetDeadband) // Add a 1% deadband
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withDeadband((kDriveToTargetMaxSpeed * SwerveConstants.kDeadband))
        .withHeadingPID(10, 0, 0);

    /** Field-centric request used by normal joystick driving. */
    public static final SwerveRequest.FieldCentric kJoystickFieldCentricRequest = new SwerveRequest.FieldCentric()
        .withDeadband(0) // DriveScalar handles the joystick deadband before shaping.
        .withRotationalDeadband(0)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    /** Robot-centric request available for driver or test modes that should ignore field heading. */
    public static final SwerveRequest.RobotCentric kJoystickRobotCentricRequest = new SwerveRequest.RobotCentric()
        .withDeadband(0)
        .withRotationalDeadband(0)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
}
