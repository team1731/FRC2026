package frc.robot.subsystems.vision.questnav;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VSLAMConstants {
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
}