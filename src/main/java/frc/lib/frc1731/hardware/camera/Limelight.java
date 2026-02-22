package frc.lib.frc1731.hardware.camera;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

public class Limelight {
    private PoseEstimator<Pose2d> estimator;
    private String name;

    public Limelight(String name) {
        this.name = name;
    }

    public void setLimelightPosition(Transform3d pose) {
        LimelightHelpers.setCameraPose_RobotSpace(
            name, 
            pose.getX(), 
            pose.getY(), 
            pose.getZ(), 
            pose.getRotation().getX(), 
            pose.getRotation().getY(), 
            pose.getRotation().getZ()
        );
    }

    public void periodic() {
        // In your periodic function:
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        if (limelightMeasurement.tagCount >= 2) {  // Only trust measurement if we see multiple tags
            estimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
            estimator.addVisionMeasurement(
                limelightMeasurement.pose,
                limelightMeasurement.timestampSeconds
            );
        }
    }

    public String getName() {
        return name;
    }

    public PoseEstimator<Pose2d> getEstimator() {
        return estimator;
    }
}