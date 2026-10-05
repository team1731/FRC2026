package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.*;
import gg.questnav.questnav.*;
import frc.lib.frc1731.SmartLogger;
import frc.lib.frc1731.hardware.camera.AprilTagIO;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Coordinates all vision pose sources and forwards accepted measurements to the drivetrain.
 *
 * <p>QuestNav/Oculus VSLAM and AprilTag cameras are intentionally handled as separate source types.
 * AprilTag cameras share the {@link AprilTagIO} interface, while QuestNav provides pose frames
 * directly through its library.
 */
public class VisionHandler {
    private QuestNav oculus;
    private AprilTagIO[] tagIOs;
    private EstimateConsumer consumer;
    private SmartLogger logger;

    /**
     * Creates a vision handler with a drivetrain pose-estimator callback and optional tag cameras.
     *
     * @param consumer callback that accepts pose, timestamp, and measurement standard deviations
     * @param ios AprilTag camera IO providers to poll when AprilTag vision is enabled
     */
    public VisionHandler(EstimateConsumer consumer, AprilTagIO... ios) {
        this.consumer = consumer;
        logger = new SmartLogger("VisionSubsystem", () -> RobotConstants.kLogTeamOutputs);
        if (VisionConstants.kUseVSLAM) {
            this.oculus = new QuestNav();
        }

        if (VisionConstants.kUseAprilTags) {
            this.tagIOs = ios;
        }
    }

    /**
     * Updates known position of the questnav vision system
     * @param pose known location to reset the questnav to
     */
    public void resetVSLAMPose(Pose2d pose) {
        if (VisionConstants.kUseVSLAM) {
            this.oculus.setPose(new Pose3d(pose).transformBy(VisionConstants.kRobotToOculus));
        }
    }

    /**
     * Resets simulated AprilTag camera pose history to match drivetrain odometry.
     *
     * @param pose robot pose to use as the new simulated camera history origin
     */
    public void resetAprilTagSimPose(Pose2d pose) {
        if (VisionConstants.kUseAprilTags) {
            for(AprilTagIO io : tagIOs) {
                io.resetSimPose(pose);
            }
        }
    }

    /**
     * Updates swerve localization based on Limelight updates. 
     * Only works if usage is enabled and limelight is fully connected and april tags visible
     */
    private void updateAprilTag(Pose2d robotPose, double yaw, double yawRate) {
        if (VisionConstants.kUseAprilTags) {
            for(AprilTagIO io : tagIOs) {
                if (Robot.isSimulation()) {
                    io.updateSimulation(robotPose);
                }
                io.periodic(yaw);

                Pose2d estimate = io.getEstimatedPose();
                boolean rejectUpdate = estimate == null
                    || !io.isConnected()
                    || io.getEstimationStdDevs() == null
                    || Math.abs(yawRate) > VisionConstants.kMaxVisionAngularRate;

                if (!rejectUpdate) {
                    double timestamp = io.getTimestamp();
                    // double age = Timer.getFPGATimestamp() - timestamp;
                    if (!Double.isFinite(timestamp) 
                            // || age < 0
                            // || age > VisionConstants.kQuestSeedMaxFrameAgeSeconds
                            || !Double.isFinite(estimate.getX()) || !Double.isFinite(estimate.getY())) {
                        continue;
                    }
                    this.consumer.accept(estimate, timestamp, io.getEstimationStdDevs());
                }
            }
        }
    }

    /**
     * Updates swerve localization based on VSLAM updates. 
     * Only works if usage is enabled and oculus is fully connected and tracking
     */
    private void updateVSLAM() {
        if (VisionConstants.kUseVSLAM) {
            oculus.commandPeriodic();

            // Get the latest pose data frames from the Quest
            PoseFrame[] questFrames = oculus.getAllUnreadPoseFrames();

            // Loop over the pose data frames and send them to the pose estimator
            for (PoseFrame questFrame : questFrames) {
                // Make sure the Quest was tracking the pose for this frame
                if (questFrame.isTracking()) {
                    // Get the pose of the Quest
                    Pose3d questPose = questFrame.questPose3d();
                    // Get timestamp for when the data was sent
                    double timestamp = questFrame.dataTimestamp();

                    // Transform by the mount pose to get your robot pose relative to the oculus start position
                    Pose3d relativeRobotPose = questPose.transformBy(VisionConstants.kRobotToOculus.inverse());

                    if (oculus.isTracking() && oculus.isConnected()) {
                        // Do not fuse an unverified Quest origin into the tag-based estimator.
                        this.consumer.accept(relativeRobotPose.toPose2d(), timestamp, VisionConstants.kQuestnavStdev);
                    }
                }
            }
        }
    }

    /**
     * Periodically updates swerve odometry given vision measurements
     * @param robotPose current drivetrain pose, used to update simulated cameras
     * @param yaw current rotation of the swerve base
     * @param yawRate current rotation velocity of the swerve base
     * @param speeds measured robot-relative speeds used to gate stationary seed checks
     */
    public void periodic(Pose2d robotPose, double yaw, double yawRate, ChassisSpeeds speeds) {
        this.updateVSLAM();
        this.updateAprilTag(robotPose, yaw, yawRate);

        // Publish every cycle, including when VSLAM is disabled or no headset is attached.
        boolean connected = oculus != null && oculus.isConnected();
        boolean tracking = connected && oculus.isTracking();

        logger.log("Oculus Connected", connected);
        logger.log("Oculus Tracking", tracking);
    }

    /**
     * Callback used by vision sources to submit a timestamped pose measurement.
     */
    @FunctionalInterface
    public static interface EstimateConsumer {
        /**
         * Accepts a pose estimate to be fused into the drivetrain pose estimator.
         *
         * @param pose estimated robot pose on the field
         * @param timestamp timestamp for when the camera observed the frame
         * @param estimationStdDevs standard deviations for x, y, and heading
         */
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}
