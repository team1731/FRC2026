package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static final Transform3d kRobotToLimelight = new Transform3d(
        Units.inchesToMeters(-7.00),
        Units.inchesToMeters(10.50),
        Units.inchesToMeters(8.75),
        new Rotation3d(
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(110.0),
            Units.degreesToRadians(180.0)
        )
    );

    public static final Transform3d kRobotToOculus = new Transform3d(
        Units.inchesToMeters(7.00),
        Units.inchesToMeters(10.50),
        Units.inchesToMeters(8.75),
        new Rotation3d(
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(0.0),
            Units.degreesToRadians(180.0)
        )
    );

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(2, 2, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.05, 0.05, 1);
    public static final Matrix<N3, N1> kTrapStdDevs = VecBuilder.fill(0.1, 0.1, 0.001); // TODO: Figure out what this does
    public static final Matrix<N3, N1> kVSLAMStdDevs = VecBuilder.fill(0.00001, 0.00001, 0.00001);

    public static final String kLimelightName = "limelight-main";
}