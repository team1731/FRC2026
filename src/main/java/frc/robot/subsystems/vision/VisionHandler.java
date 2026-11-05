package frc.robot.subsystems.vision;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.frc1731.SmartLogger;
import frc.lib.frc1731.hardware.camera.AprilTagIO;
import frc.robot.Robot;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class VisionHandler {
    private QuestNav oculus;
    private AprilTagIO[] tagIOs;
    private SmartLogger logger;

    private boolean isQuestSeeded = false;

    public VisionHandler(AprilTagIO... ios) {
        this.oculus = VisionConstants.kUseVSLAM ? new QuestNav() : null;
        if (VisionConstants.kUseAprilTags) {
            this.tagIOs = ios;
        } else {
            this.tagIOs = new AprilTagIO[]{};
        }

        logger = new SmartLogger("Vision", () -> true);
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
            isQuestSeeded = true;
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

    public void periodic() {
        updateVSLAM();
        updateAprilTag();
    }

    private void updateAprilTag() {
        if (!VisionConstants.kUseAprilTags
                || (oculus != null && oculus.isConnected() && oculus.isTracking())) {
            return;
        }

        for (AprilTagIO io : tagIOs) {
            if (Robot.isSimulation()) io.updateSimulation(Robot.swerve.getPose());
            Pigeon2 gyro = Robot.swerve.getPigeon();
            io.periodic(
                gyro.getYaw().getValueAsDouble(),
                gyro.getAngularVelocityZDevice().getValueAsDouble(),
                gyro.getPitch().getValueAsDouble(),
                gyro.getAngularVelocityYDevice().getValueAsDouble(),
                gyro.getRoll().getValueAsDouble(),
                gyro.getAngularVelocityXDevice().getValueAsDouble()
            );

            Pose2d estimate = io.getEstimatedPose();
            boolean rejectUpdate = estimate == null
                    || !io.isConnected()
                    || io.getEstimationStdDevs() == null
                    || io.getTagDistance() > 5.0
                    || Math.abs(gyro.getAngularVelocityZDevice().getValueAsDouble()) > VisionConstants.kMaxVisionAngularRate;

            if (!rejectUpdate) {
                if (DriverStation.isDisabled()) {
                    // Align the physical gyro and the estimator's field heading together.
                    Robot.swerve.resetYaw(estimate.getRotation());
                    Robot.swerve.resetPose(estimate);
                } else {
                    // Tags are only polled while Quest is unavailable or not tracking
                    Robot.swerve.addVisionMeasurement(estimate, io.getTimestamp(), io.getEstimationStdDevs());
                }
            }
        }
    }

    private void updateVSLAM() {
        if (oculus != null) {
            oculus.commandPeriodic();
            boolean connected = oculus.isConnected();
            boolean tracking = connected && oculus.isTracking();

            // Get the latest pose data frames from the Quest
            PoseFrame[] questFrames = oculus.getAllUnreadPoseFrames();
            if (questFrames.length > 0) {
                PoseFrame latestFrame = questFrames[questFrames.length - 1];
                Pose3d oculusPose = latestFrame.questPose3d();
                Pose2d swervePose = oculusPose.transformBy(VisionConstants.kRobotToOculus.inverse()).toPose2d();
                if (tracking) {
                    Robot.swerve.addVisionMeasurement(swervePose, latestFrame.dataTimestamp(), VisionConstants.kQuestnavStdev);
                } else {
                    isQuestSeeded = false;
                }
                logger.log("Oculus Pose", swervePose);
            }

            logger.log("Oculus Tracking", tracking);
            logger.log("Oculus Connected", connected);
            logger.log("Oculus Seeded", isQuestSeeded);
            oculus.getBatteryPercent()
                .ifPresentOrElse(
                    percent -> logger.log("Oculus Battery", percent),
                    () -> logger.log("Oculus Battery", 0)
                );
        }
    }
}