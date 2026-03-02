package frc.robot.subsystems.vision.limelight;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class AprilTagConstants {
    public static final Transform3d kCameraToRobot = new Transform3d(
        Units.inchesToMeters(-7.00),
        Units.inchesToMeters(10.50),
        Units.inchesToMeters(8.75),
        new Rotation3d(
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(180.0)
        )
    );
    public static final String kLimelightName = "limelight-main";
}