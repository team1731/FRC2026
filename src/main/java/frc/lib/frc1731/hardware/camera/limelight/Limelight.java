package frc.lib.frc1731.hardware.camera.limelight;

import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.frc1731.hardware.camera.limelight.LimelightHelpers.PoseEstimate;

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

    public PoseEstimate getMT2Estimate(double yaw) {
        LimelightHelpers.SetRobotOrientation(name, yaw, 0, 0, 0, 0, 0);
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    }

    public PoseEstimate getMT1Estimate() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    }

    public String getName() {
        return name;
    }

    public double getHeartbeat() {
        return LimelightHelpers.getHeartbeat(name);
    }
}