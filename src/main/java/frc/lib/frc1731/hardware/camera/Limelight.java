package frc.lib.frc1731.hardware.camera;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.frc1731.hardware.camera.LimelightHelpers.PoseEstimate;

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

    public void updateOrientation(Pigeon2 pigeon) {
        LimelightHelpers.SetRobotOrientation(
            name,
            pigeon.getYaw().getValueAsDouble(), 
            pigeon.getAngularVelocityZDevice().getValueAsDouble(), 
            pigeon.getPitch().getValueAsDouble(), 
            pigeon.getAngularVelocityXDevice().getValueAsDouble(),  
            pigeon.getRoll().getValueAsDouble(), 
            pigeon.getAngularVelocityYDevice().getValueAsDouble()
        );
    }

    public PoseEstimate getPoseEstimate() {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    }

    public Pose2d getBotPose() {
        return LimelightHelpers.getBotPose2d(name);
    }

    public String getName() {
        return name;
    }
}