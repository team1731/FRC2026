package frc.lib.frc1731.hardware.camera;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.vision.VisionConstants;

/**
 * Common interface for AprilTag-based pose-estimation cameras.
 *
 * <p>Implementations should cache their latest valid estimate during {@link #periodic(double)} and
 * expose that estimate through the getters. This lets {@code VisionHandler} handle Limelight,
 * PhotonVision, and future tag cameras the same way.
 */
public interface AprilTagIO {
    /**
     * Returns the human-readable camera name used in NetworkTables and logs.
     *
     * @return camera name
     */
    public String getName();

    /**
     * Reports whether this camera source is connected and producing data.
     *
     * @return true when the camera connection is healthy
     */
    public boolean isConnected();

    /**
     * Returns standard deviations for the latest pose estimate.
     *
     * @return measurement trust values for x, y, and heading
     */
    public Matrix<N3, N1> getEstimationStdDevs();

    /**
     * Returns the timestamp for the latest pose estimate.
     *
     * @return camera frame timestamp in seconds
     */
    public double getTimestamp();

    /**
     * Returns the latest estimated robot pose, or {@code null} if no usable estimate exists.
     *
     * @return latest estimated field pose
     */
    public Pose2d getEstimatedPose();

    /**
     * Resets pose history of the robot in the vision system simulation.
     *
     * @param robotPose pose to use when resetting simulated vision state
     */
    public void resetSimPose(Pose2d robotPose);

    /**
     * Polls the camera and updates the latest cached estimate.
     *
     * @param yaw current robot yaw in degrees
     */
    public void periodic(double yaw);

    /**
     * Updates the simulated camera scene from the robot's simulated pose.
     *
     * @param robotPose current simulated robot pose
     */
    public void updateSimulation(Pose2d robotPose);

    /**
     * Whether we want the cameras to update swerve odometry from april tag data
     */
    public default boolean isActive() {
        return VisionConstants.kUseAprilTags;
    }
}
