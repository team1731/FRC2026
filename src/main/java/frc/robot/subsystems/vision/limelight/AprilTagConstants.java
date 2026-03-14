package frc.robot.subsystems.vision.limelight;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class AprilTagConstants {
    public static final Transform3d kCameraToRobot = new Transform3d(
        Units.inchesToMeters(-11.25),
        Units.inchesToMeters(9.5),
        Units.inchesToMeters(13.25
        ),
        new Rotation3d(
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(110.0),
            Units.degreesToRadians(180.0)
        )
    );
    public static final String kLimelightName = "limelight-main";
}