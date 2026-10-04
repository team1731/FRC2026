package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import frc.lib.frc1731.DriveScalar;
import frc.lib.frc1731.DriveScalar.ScaleType;
import frc.lib.frc1731.PIDGains;
import frc.robot.subsystems.swerve.generated.TunerConstants;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;

/**
 * Tunable constants for the swerve subsystem, autonomous pathing, and auto-alignment behavior.
 */
public final class SwerveConstants {
    // Speed constants and scaling
    /** Maximum translational speed of the drivetrain in meters per second. */
    public static final double kMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // Max robot speed

    /** Maximum commanded angular speed in radians per second. */
    public static final double kMaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /** Maximum commanded angular acceleration in radians per second squared. */
    public static final double kMaxAngularAcceleration = Math.toRadians(540); // deg/s^2 max acceleration

    /** Scalar applied to translation joystick commands. */
    public static final double kTranslationScalar = 1.0;

    /** Scalar applied to rotation joystick commands. */
    public static final double kRotationScalar = 0.6;

    /** Joystick deadband before shaping/scaling inputs. */
    public static final double kDeadband = 0.05; // 5% joystick deadband

    // Current limits
    /** Drive stator current limit used during autonomous. */
    public static final double kAutoCurrentLimit = 100;

    /** Drive stator current limit used during teleop and test. */
    public static final double kTeleCurrentLimit = 60;

    // PID Gains
    /** Heading controller gains for field-centric heading correction and auto-align rotation. */
    public static final PIDGains kHeadingGains = new PIDGains()
        .setP(0.5)
        .setD(0.05)
        .setTolerance(1.0) // 1 degree tolerance
        .setContinuousInput(-180, 180); // 4 m/s when 180 degrees of error

    /** Translation controller gains used when driving to an exact field position. */
    public static final PIDGains kDriveAtTargetGains = new PIDGains()
        .setP(10)
        .setTolerance(0.02); // 2 cm tolerance

    // PID Controllers
    /** PID controller for auto-align left/right field correction. */
    public static final PIDController kAutoAlignStrafeCtrl = kDriveAtTargetGains.toPIDController();

    /** PID controller for auto-align forward/backward field correction. */
    public static final PIDController kAutoAlignDriveCtrl = kDriveAtTargetGains.toPIDController();

    /** PID controller for maintaining or driving to a requested heading. */
    public static final PIDController kHeadingCtrl = kHeadingGains.toPIDController();

    // Pathing Constraints
    /** Shared PathPlanner translation and rotation PID constants. */
    public static final PIDConstants kPPConstants = new PIDConstants(10d, 0d, 0d); // PID constants for PathPlanner path following

    /** Motion limits used when PathPlanner computes on-the-fly paths. */
    public static final PathConstraints kPathfinderConstraints = new PathConstraints(
        kMaxSpeed, kMaxSpeed * 2.0,
        kMaxAngularRate, kMaxAngularRate * 1.5
    );

    // Scalars
    /** Driver forward/backward input shaper. */
    public static final DriveScalar kXScalar = new DriveScalar(ScaleType.kQuadratic, kDeadband);

    /** Driver left/right input shaper. */
    public static final DriveScalar kYScalar = new DriveScalar(ScaleType.kQuadratic, kDeadband);

    /** Driver rotation input shaper. */
    public static final DriveScalar kOmegaScalar = new DriveScalar(ScaleType.kQuadratic, kDeadband);
}
