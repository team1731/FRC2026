package frc.robot.subsystems.vision;

import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.util.Units;
import frc.lib.frc1731.hardware.camera.*;

/**
 * Vision feature flags, camera names, camera transforms, and pose-estimator trust settings.
 */
public final class VisionConstants {
    /** Enables QuestNav/Oculus VSLAM measurements. */
    public static final boolean kUseVSLAM = true;

    /** Enables AprilTag camera measurements from configured {@link AprilTagIO} providers. */
    public static final boolean kUseAprilTags = true;

    public static final double kQuestSeedMaxFrameAgeSeconds = 0.25;
    public static final double kQuestSeedMaxAngularRate = 30.0; // degrees per second
    public static final double kQuestSeedMaxSpeed = 0.25; // meters per second
    public static final double kQuestSeedStableSeconds = 0.2;
    public static final int kQuestSeedMinObservations = 3;
    public static final double kQuestSeedMaxObservationGapSeconds = 0.15;
    public static final double kQuestSeedMaxPositionSpreadMeters = 0.1;
    public static final double kQuestSeedMaxHeadingSpreadDegrees = 10.0;

    
    /** NetworkTables name for the primary Limelight. */
    public static final String kLimelightMainName = "limelight-main";

    /** Oculus battery percentage threshold for low-battery warnings. */
    public static final int kOculusLowBattery = 20;

    /** Maximum robot yaw rate where AprilTag pose updates are trusted. */
    public static final double kMaxVisionAngularRate = 360; // degrees per second

    /** Transform from robot origin to the mounted Quest/Oculus headset. */
    public static final Transform3d kRobotToOculus = new Transform3d(
        Units.inchesToMeters(-12.25),
        Units.inchesToMeters(0.5),
        Units.inchesToMeters(12.5),
        new Rotation3d(
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(180.0)
        )
    );

    public static final Transform3d kRobotToLimelightMain = new Transform3d(
        Units.inchesToMeters(-12),
        Units.inchesToMeters(0.00),
        Units.inchesToMeters(17.25),
        new Rotation3d(
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(-20.0),
            Units.degreesToRadians(180.0)
        )
    );

    // Note from Brent - these were way way smaller last year
    /** Standard deviations for QuestNav/Oculus pose measurements: x, y, theta. */
    public static final Matrix<N3, N1> kQuestnavStdev = VecBuilder.fill(0.01, 0.01, 0.02);

    /** Baseline standard deviations for single-tag PhotonVision estimates. */
    public static final Matrix<N3, N1> kPhotonVisionSingleTagStdev = VecBuilder.fill(0.8, 0.8, 999999);

    /** Baseline standard deviations for multi-tag PhotonVision estimates. */
    public static final Matrix<N3, N1> kPhotonVisionMultiTagStdev = VecBuilder.fill(0.2, 0.2, 999999);

    /**
     * Builds the AprilTag IO providers that should feed robot pose estimation.
     *
     * @return all configured AprilTag camera IO implementations
     */
    public static AprilTagIO[] getAprilTagIOs() {
        return new AprilTagIO[]{
            new LimelightIO(kLimelightMainName, kRobotToLimelightMain)
        };
    }
}
