package frc.robot.subsystems.vision;

import frc.lib.frc1731.hardware.camera.AprilTagIO;
import frc.lib.frc1731.subsystem.BaseSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionHandler.EstimateConsumer;
import gg.questnav.questnav.QuestNav;

public class NewVisionHandler {
    private SwerveSubsystem swerve;
    private QuestNav oculus;
    private AprilTagIO[] tagIOs;

    private boolean isQuestSeeded = false;

    /**
     * Creates a vision handler with a drivetrain pose-estimator callback and optional tag cameras.
     *
     * @param consumer callback that accepts pose, timestamp, and measurement standard deviations
     * @param ios AprilTag camera IO providers to poll when AprilTag vision is enabled
     */
    public NewVisionHandler(SwerveSubsystem swerve, AprilTagIO... ios) {
        this.swerve = swerve;
        // logger = new SmartLogger("VisionSubsystem", () -> RobotConstants.kLogTeamOutputs);
        if (VisionConstants.kUseVSLAM) {
            this.oculus = new QuestNav();
        }

        if (VisionConstants.kUseAprilTags) {
            this.tagIOs = ios;
        } else {
            this.tagIOs = new AprilTagIO[]{};
        }
    }
}