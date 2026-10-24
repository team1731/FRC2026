package frc.robot.subsystems.vision;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import gg.questnav.questnav.*;
import frc.lib.frc1731.hardware.camera.AprilTagIO;
import frc.robot.Robot;

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

    private boolean isQuestSeeded = false;
    private double questResetTimestamp = Double.NEGATIVE_INFINITY;
    private final Map<AprilTagIO, Double> lastTagTimestamps = new HashMap<>();
    private final Map<AprilTagIO, Pose2d> questSeedFirstPoses = new HashMap<>();
    private final Map<AprilTagIO, Double> questSeedFirstTimestamps = new HashMap<>();
    private final Map<AprilTagIO, Double> questSeedLastTimestamps = new HashMap<>();
    private final Map<AprilTagIO, Integer> questSeedObservationCounts = new HashMap<>();
    private double questSeedMotionStableSince = Double.NaN;

    /**
     * Creates a vision handler with a drivetrain pose-estimator callback and optional tag cameras.
     *
     * @param consumer callback that accepts pose, timestamp, and measurement standard deviations
     * @param ios AprilTag camera IO providers to poll when AprilTag vision is enabled
     */
    public VisionHandler(EstimateConsumer consumer, AprilTagIO... ios) {
        this.consumer = consumer;
        this.oculus = VisionConstants.kUseVSLAM ? new QuestNav() : null;

        if (VisionConstants.kUseAprilTags) {
            this.tagIOs = ios;
        } else {
            this.tagIOs = new AprilTagIO[]{};
        }
    }

    /**
     * Sets the known Quest field position and immediately marks it seeded.
     * @param pose known location to reset the questnav to
     */
    public void resetVSLAMPose(Pose2d pose) {
        if (oculus != null) {
            // Discard queued measurements expressed in the previous field reference.
            oculus.getAllUnreadPoseFrames();
            this.oculus.setPose(new Pose3d(pose).transformBy(VisionConstants.kRobotToOculus));
            questResetTimestamp = Timer.getFPGATimestamp();
            isQuestSeeded = true;
            resetQuestSeedWindows();
        }
    }

    public boolean isQuestSeeded() {
        return isQuestSeeded;
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
    private void updateAprilTag(Pose2d robotPose, double yaw, double yawRate, double speed) {
        // Quest connection/tracking controls source selection, independently of seed state.
        // Return before polling cameras or running their simulation/pose-estimation work.
        if (!VisionConstants.kUseAprilTags
                || (oculus != null && oculus.isConnected() && oculus.isTracking())) {
            return;
        }
        // boolean canSeedQuest = DriverStation.isTeleopEnabled() && oculus != null
        //     && oculus.isConnected() && oculus.isTracking() && !isQuestSeeded
        //     && isQuestSeedMotionSlowEnough(speed, yawRate);
        // if (!canSeedQuest) {
        //     resetQuestSeedWindows();
        //     questSeedMotionStableSince = Double.NaN;
        // } else if (Double.isNaN(questSeedMotionStableSince)) {
        //     questSeedMotionStableSince = Timer.getFPGATimestamp();
        // }

        if (VisionConstants.kUseAprilTags) {
            for(AprilTagIO io : tagIOs) {
                if (Robot.isSimulation()) {
                    io.updateSimulation(robotPose);
                }

                io.periodic(yaw, yawRate);

                Pose2d estimate = io.getEstimatedPose();
                boolean rejectUpdate = estimate == null
                    || !io.isConnected()
                    || io.getEstimationStdDevs() == null
                    || io.getTagDistance() > 3.0
                    || Math.abs(yawRate) > VisionConstants.kMaxVisionAngularRate;

                if (!rejectUpdate) {
                    double timestamp = io.getTimestamp();
                    if (!Double.isFinite(timestamp) 
                            || !Double.isFinite(estimate.getX()) || !Double.isFinite(estimate.getY())
                            || !Double.isFinite(estimate.getRotation().getRadians())) {
                        resetQuestSeedWindow(io);
                        continue;
                    }
                    double age = Timer.getFPGATimestamp() - timestamp;
                    if (age < 0 || age > VisionConstants.kQuestSeedMaxFrameAgeSeconds) {
                        resetQuestSeedWindow(io);
                        continue;
                    }
                    if (timestamp <= lastTagTimestamps.getOrDefault(io, Double.NEGATIVE_INFINITY)) {
                        continue;
                    }
                    lastTagTimestamps.put(io, timestamp);
                    if (DriverStation.isDisabled()) {
                        // Align the physical gyro and the estimator's field heading together.
                        // Robot.swerve.resetYaw(estimate.getRotation());
                        // Robot.swerve.resetPose(estimate);
                    } else {
                        // Tags are only polled while Quest is unavailable or not tracking.
                        this.consumer.accept(estimate, timestamp, io.getEstimationStdDevs());
                        // if (canSeedQuest && !isQuestSeeded && hasStableQuestSeed(io, estimate, timestamp)
                        //         && Timer.getFPGATimestamp() - questSeedMotionStableSince
                        //             >= VisionConstants.kQuestSeedStableSeconds) {
                        //     // Enabled tag updates seed Quest only; they never hard-reset swerve.
                        //     resetVSLAMPose(estimate);
                        // }
                    }
                } else {
                    resetQuestSeedWindow(io);
                }
            }
        }
    }

    /**
     * Updates swerve localization based on VSLAM updates. 
     * Only works if usage is enabled and oculus is fully connected and tracking
     */
    private void updateVSLAM() {
        if (oculus != null) {
            oculus.commandPeriodic();
            boolean connected = oculus.isConnected();
            boolean tracking = connected && oculus.isTracking();
            if (!tracking) invalidateQuestSeed();

            // Get the latest pose data frames from the Quest
            PoseFrame[] questFrames = oculus.getAllUnreadPoseFrames();

            // A loss anywhere in the queued history invalidates the field reference.
            for (PoseFrame questFrame : questFrames) {
                if (!questFrame.isTracking()) {
                    invalidateQuestSeed();
                    continue;
                }
                if (isQuestSeeded && tracking) {
                    // Get the pose of the Quest
                    Pose3d questPose = questFrame.questPose3d();
                    // Get timestamp for when the data was sent
                    double timestamp = questFrame.dataTimestamp();
                    if (!Double.isFinite(timestamp) || timestamp <= questResetTimestamp) continue;

                    // Transform by the mount pose to get your robot pose relative to the oculus start position
                    Pose3d relativeRobotPose = questPose.transformBy(VisionConstants.kRobotToOculus.inverse());

                    if (oculus.isTracking() && oculus.isConnected()) {
                        this.consumer.accept(relativeRobotPose.toPose2d(), timestamp, VisionConstants.kQuestnavStdev);
                    } else {
                        invalidateQuestSeed();
                    }
                }
            }


            SmartDashboard.putBoolean("Oculus Tracking", tracking);
            SmartDashboard.putBoolean("Oculus Connected", connected);
            SmartDashboard.putBoolean("Oculus Seeded", isQuestSeeded);
            oculus.getBatteryPercent()
                .ifPresentOrElse(
                    percent -> SmartDashboard.putNumber("Oculus Battery", percent),
                    () -> SmartDashboard.putNumber("Oculus Battery", 0)
                );
        }
    }

    static boolean isQuestSeedMotionSlowEnough(double speed, double yawRate) {
        return Double.isFinite(speed) && speed >= 0 && speed < VisionConstants.kQuestSeedMaxSpeed
            && Double.isFinite(yawRate) && Math.abs(yawRate) < VisionConstants.kQuestSeedMaxAngularRate;
    }

    private void resetQuestSeedWindow(AprilTagIO io) {
        questSeedFirstPoses.remove(io);
        questSeedFirstTimestamps.remove(io);
        questSeedLastTimestamps.remove(io);
        questSeedObservationCounts.remove(io);
    }

    private void resetQuestSeedWindows() {
        questSeedFirstPoses.clear();
        questSeedFirstTimestamps.clear();
        questSeedLastTimestamps.clear();
        questSeedObservationCounts.clear();
    }

    /** Requires consecutive, consistent observations from the same camera before teleop seeding. */
    private boolean hasStableQuestSeed(AprilTagIO io, Pose2d pose, double timestamp) {
        Pose2d firstPose = questSeedFirstPoses.get(io);
        double lastTimestamp = questSeedLastTimestamps.getOrDefault(io, Double.NEGATIVE_INFINITY);
        if (firstPose != null && timestamp <= lastTimestamp) return false;
        if (firstPose == null
                || timestamp - lastTimestamp > VisionConstants.kQuestSeedMaxObservationGapSeconds
                || pose.getTranslation().getDistance(firstPose.getTranslation())
                    > VisionConstants.kQuestSeedMaxPositionSpreadMeters
                || Math.abs(pose.getRotation().minus(firstPose.getRotation()).getDegrees())
                    > VisionConstants.kQuestSeedMaxHeadingSpreadDegrees) {
            questSeedFirstPoses.put(io, pose);
            questSeedFirstTimestamps.put(io, timestamp);
            questSeedObservationCounts.put(io, 0);
        }
        questSeedLastTimestamps.put(io, timestamp);
        int observations = questSeedObservationCounts.merge(io, 1, Integer::sum);
        return observations >= VisionConstants.kQuestSeedMinObservations
            && timestamp - questSeedFirstTimestamps.get(io) >= VisionConstants.kQuestSeedStableSeconds;
    }

    private void invalidateQuestSeed() {
        isQuestSeeded = false;
        questSeedMotionStableSince = Double.NaN;
        resetQuestSeedWindows();
    }

    /**
     * Periodically updates swerve odometry given vision measurements
     * @param robotPose current drivetrain pose, used to update simulated cameras
     * @param yaw current rotation of the swerve base
     * @param yawRate current rotation velocity of the swerve base
     * @param speed measured robot translation speed in meters per second
     */
    public void periodic(Pose2d robotPose, double yaw, double yawRate, double speed) {
        this.updateVSLAM();
        this.updateAprilTag(robotPose, yaw, yawRate, speed);
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

