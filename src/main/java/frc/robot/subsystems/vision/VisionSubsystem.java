package frc.robot.subsystems.vision;

import frc.lib.frc1731.hardware.camera.limelight.*;
import frc.lib.frc1731.hardware.camera.limelight.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.BaseSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class VisionSubsystem extends BaseSubsystem {
    private Limelight limelight;
    private QuestNav oculus;

    private SwerveSubsystem swerve;

    private boolean isQuestSeeded = false;

    private Pose2d baselinePose = new Pose2d();

    private Timer goodLimelightTimer = new Timer();
    private Timer questPoseResetTimer = new Timer();  // ussed primarily in dissabled to reset the pose every few seconds in case the robot is moved b4 auto

    private PoseEstimate estimate = null;

    public VisionSubsystem(SwerveSubsystem swerve, boolean enabled) {
        super(enabled);
        this.swerve = swerve;
    }

    @Override
    public void initializeHardware() {
        if (kUseVSLAM) {
            oculus = new QuestNav();
        }
        
        if (kUseAprilTags) {
            this.limelight = new Limelight(kLimelightName);
            this.limelight.setLimelightPosition(kLimelightToRobot);
        }
    }

    public void resetOculusPose(Pose3d pose) {
        if (kUseVSLAM) {
            this.oculus.setPose(pose);
        }
    }

    public boolean isVSLAMConnected() {
        if (!kUseAprilTags) return false;
        return oculus.isConnected() && oculus.isTracking();
    }

    private void addQuestVisionMeasurement() {
        if (!kUseVSLAM) return;
        
        questPoseResetTimer.start();

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

                // Transform by the mount pose to get your robot pose
                Pose3d robotPose = questPose.transformBy(kRobotToOculus.inverse());

                if (isVSLAMConnected() && isQuestSeeded) {
                    swerve.addVisionMeasurement(robotPose.toPose2d(), timestamp, kQuestnavStdev);
                }
            }
        }
    }

    @Override
    public void periodicTelemetry() {
        if (kUseAprilTags) {
            estimate = limelight.getMT2Estimate(swerve.getYaw());
            if (estimate.tagCount > 1) {
                goodLimelightTimer.start();
                if (goodLimelightTimer.hasElapsed(2) && 
                    estimate.pose.minus(baselinePose).getTranslation().getNorm() < 0.005) {
                        swerve.resetPose(baselinePose);
                        goodLimelightTimer.restart();
                        if (kUseVSLAM) {
                            resetOculusPose(new Pose3d(estimate.pose));
                            isQuestSeeded = true;
                        }
                }
            } else {
                goodLimelightTimer.restart();
            }

            baselinePose = estimate.pose.rotateBy(Rotation2d.k180deg);
        }
        
        if (kUseVSLAM) {
            questPoseResetTimer.start();
            if (!isQuestSeeded || (DriverStation.isDisabled() && questPoseResetTimer.hasElapsed(5))) {
                if (isVSLAMConnected()) {
                    this.resetOculusPose(new Pose3d(swerve.getCurrentPose()));
                    questPoseResetTimer.restart();
                    isQuestSeeded = true;
                }
            }

            if (!oculus.isTracking()) {
                isQuestSeeded = false;
            }
            
            if (isVSLAMConnected() && isQuestSeeded) {
                addQuestVisionMeasurement();
            }
        }
    }
}