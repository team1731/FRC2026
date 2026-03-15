package frc.lib.frc1731.field;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;

public enum FieldPositions {
    kHub(4.625594, 4.034536, true),
    kLeftPass(2, 7, false),
    kRightPass(2, 1, false),
    ;

    public static final double kFieldLength = 16.518;
    public static final double kFieldWidth = 8.043;

    private final Translation2d position;
    private final boolean flipY;
    FieldPositions(double blueX, double blueY, boolean flipY) {
        this.position = new Translation2d(blueX, blueY);
        this.flipY = flipY;
    }
    
    public Translation2d get() {
        double x = Robot.isRedAlliance() ? kFieldLength - position.getX() : position.getX();
        double y = Robot.isRedAlliance() && flipY ? kFieldWidth - position.getY() : position.getY();
        return new Translation2d(x, y);
    }

    public Translation2d getRaw() {
        return position;
    }
}