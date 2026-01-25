package frc.lib.frc1731.field;

import edu.wpi.first.math.geometry.*;
import frc.robot.Robot;

public enum Field {
    HUB(new Translation3d(11.950, 4.024, 2.67), new Translation3d(4.637, 4.024, 2.67)),
    DEPOT(new Translation3d(), new Translation3d()),
    OUTPOST(new Translation3d(), new Translation3d()),
    TOWER_LEFT(new Translation3d(), new Translation3d()),
    TOWER_CENTER(new Translation3d(), new Translation3d()),
    TOWER_RIGHT(new Translation3d(), new Translation3d()),
    PASS_LEFT(new Translation3d(), new Translation3d()),
    PASS_RIGHT(new Translation3d(), new Translation3d()),
    PASS_CENTER(new Translation3d(), new Translation3d())
    ;

    private final Translation3d redPosition, bluePosition;
    private Field(Translation3d redPosition, Translation3d bluePosition) {
        this.redPosition = redPosition;
        this.bluePosition = bluePosition;
    }

    public Translation3d getPosition() {
        if (Robot.isRedAlliance()) return redPosition;
        return bluePosition;
    }
}