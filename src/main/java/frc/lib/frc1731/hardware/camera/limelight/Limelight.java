package frc.lib.frc1731.hardware.camera.limelight;

import edu.wpi.first.math.geometry.Transform3d;

public class Limelight {
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
      //  LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-main");
      //  if (limelightMeasurement.tagCount >= 2) {  // Only trust measurement if we see multiple tags
            // estimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
            // estimator.addVisionMeasurement(
            //     limelightMeasurement.pose,
            //     limelightMeasurement.timestampSeconds
            // );
     //   }
    }

    public String getName() {
        return name;
    }

    // public Optional<Pose2d> getEstimatedPose() {
    //     // PoseEstimate estimate;
    //     // if (Robot.isRedAlliance()) {
    //     //     estimate = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(name);
    //     // } else {
    //     //     estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    //     // }
    //     Pose2d pose = LimelightHelpers.get(name);
    //     Logger.recordOutput("Estimated Pose Pose Pose", pose);
    //     if (pose == null) return Optional.empty();
    //     return Optional.of(pose);
    //     // return Optional.of(estimate.pose);
    // }
}