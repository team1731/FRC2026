package frc.lib.frc1731.field;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;

public enum FieldPositions {
    kHub(4.625594, 4.034536),
    kLeftPass(3, 5),
    kRightPass(3, 3),
    ;

    public static final double kFieldLength = 16.518;
    public static final double kFieldWidth = 8.043;

    private final Translation2d position;
    FieldPositions(double blueX, double blueY) {
        this.position = new Translation2d(blueX, blueY);
    }
    
    public Translation2d get() {
        return Robot.isRedAlliance() ? 
            new Translation2d(kFieldLength, kFieldWidth).minus(position) : 
            position;
    }

    public Translation2d getRaw() {
        return position;
    }
}