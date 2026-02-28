package frc.robot.subsystems.vision.limelight;

import frc.robot.subsystems.BaseSubsystem;

import static frc.robot.subsystems.vision.limelight.AprilTagConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.frc1731.hardware.camera.Limelight;

public class AprilTagSubsystem extends BaseSubsystem {
    private Limelight limelight;

    public AprilTagSubsystem(boolean enabled) {
        super(enabled);
        this.limelight = new Limelight(kLimelightName);
        this.limelight.setLimelightPosition(kCameraToRobot);
    }

    @Override
    public void periodicTelemetry() {
        limelight.periodic();
        logger.log("Estimated Robot Pose", getEstimatedPose());
    }

    public Pose2d getEstimatedPose() {
        if (limelight.getEstimatedPose().isEmpty()) return new Pose2d();
        return limelight.getEstimatedPose().get();
    }
}