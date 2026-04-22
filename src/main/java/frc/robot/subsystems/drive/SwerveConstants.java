package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.lib.frc1731.PIDGains;
import frc.robot.subsystems.drive.generated.TunerConstants;

public class SwerveConstants {
    public static final double kMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // Max robot speed
    public static final double kMaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    public static final double kMaxAngularAcceleration = Math.toRadians(540); // deg/s^2 max acceleration
    public static final double kDeadband = 0.05; // 5% joystick deadband
    public static final double kMaxVisionAngularRate = 720d; // degrees per second

    public static final boolean kShouldTelemetrize = true;

    public static final PIDConstants kPPConstants = new PIDConstants(10d, 0d, 0d); // PID constants for PathPlanner path following

    public static final PIDGains kDriveAtTargetGains = new PIDGains().setP(10).setContinuousInput(-Math.PI, Math.PI);

    public static final double kHeadingTolerance = 1.0; // degrees;

    public static final Matrix<N3, N1> kEstimatorStateStdev = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
    public static final Matrix<N3, N1> kEstimatorMeasurementStdev = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

    public static final PIDGains kHeadingGains = new PIDGains()
        // .setP(4d/180d); // 4 m/s when 180 degrees of error
        .setP(6.0)
        .setD(0.0)
        // .setD(0.0001d)
        .setContinuousInput(-360, 360); // 4 m/s when 180 degrees of error

    public static final PathConstraints kPathfinderConstraints = new PathConstraints(
        2.0, 2.0,
        Units.degreesToRadians(360), Units.degreesToRadians(540)
    );

    /* Setting up bindings for necessary control of the swerve drive platform */
    public static final SwerveRequest.FieldCentric kFieldCentricControl = new SwerveRequest.FieldCentric()
        .withDeadband(kMaxSpeed * kDeadband) // Add a 5% deadband
        .withRotationalDeadband(kMaxAngularRate * kDeadband) // Add a 5% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    public static final SwerveRequest.RobotCentric kRobotCentricControl = new SwerveRequest.RobotCentric()
        .withDeadband(kMaxSpeed * kDeadband) // Add a 5% deadband
        .withRotationalDeadband(kMaxAngularRate * kDeadband) // Add a 5% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static final double kDriveToTargetMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private static final double kDriveToTargetDeadband = 0.01; // Add a 1% deadband

    public static final SwerveRequest.FieldCentricFacingAngle driveAtTargetControl = new SwerveRequest.FieldCentricFacingAngle()
        .withRotationalDeadband(SwerveConstants.kMaxAngularRate * kDriveToTargetDeadband) // Add a 1% deadband
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withDeadband((kDriveToTargetMaxSpeed * kDeadband));

    static {
        driveAtTargetControl.HeadingController.setPID(10, 0, 0);
        driveAtTargetControl.HeadingController.enableContinuousInput(-Math.PI/2, Math.PI/2);
    }
    
    public static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    public static final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();

    public static final double kAutoCurrentLimit = 100;
    public static final double kTeleCurrentLimit = 60;
}