package frc.robot.subsystems.vision;

import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static final boolean kUseVSLAM = false;
    public static final boolean kUseAprilTags = false;
    
    public static final String kLimelightName = "limelight-main";

    public static final Transform3d kRobotToOculus = new Transform3d(
        Units.inchesToMeters(-12.5),
        Units.inchesToMeters(-7.5),
        Units.inchesToMeters(14.5),
        new Rotation3d(
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(180.0)
        )
    );

    public static final Transform3d kLimelightToRobot = new Transform3d(
        Units.inchesToMeters(-11.25),
        Units.inchesToMeters(-9.5),
        Units.inchesToMeters(13.25),
        new Rotation3d(
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(110.0),
            Units.degreesToRadians(180.0)
        )
    );

    public static final Matrix<N3, N1> kQuestnavStdev = VecBuilder.fill(
        0.02, // Trust down to 2cm in X direction
        0.02, // Trust down to 2cm in Y direction
        0.035 // Trust down to 2 degrees rotational
    );

    public static final Matrix<N3, N1> kLimelightStdev = VecBuilder.fill(.5,.5,999999);
}