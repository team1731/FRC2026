package frc.robot.shooting;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.*;
import frc.lib.frc1731.field.FieldPositions;
import frc.robot.Robot;

public class ShotGenerator {
    public final ShotTable shotTable = ShotTable.getScoringTable();
    public final ShotTable passingTable = ShotTable.getPassingTable();
    private ShotTable activeShotTable = shotTable;

    public final Supplier<Translation2d> kHubSupplier = () -> Robot.isRedAlliance()
        ? new Translation2d(11.91, 4.03)
        : new Translation2d(4.62, 4.03);

    public final Supplier<Translation2d> kPassSupplier = () -> {
        double x = Robot.isRedAlliance() ? FieldPositions.kFieldLength - 2 : 2;
        double y = Robot.isRedAlliance() && Robot.swerve.getPose().getY() > FieldPositions.kFieldWidth / 2.0 ? FieldPositions.kFieldWidth - 2 : 2;
        return new Translation2d(x, y);
    };

    public Translation2d compensatedTarget  = new Translation2d();
    public Supplier<Translation2d> targetSupplier = kHubSupplier;
    public final Supplier<Translation2d> appliedTargetSupplier  = () -> compensatedTarget;

    public Angle targetHoodAngle = Rotations.zero();
    public AngularVelocity targetFlywheelVelocity = RotationsPerSecond.zero();
    public boolean adjustTargetForMovingShots = false;

    /**
     * Convenience wrapper: extract time-of-flight from the shot table.
     * Keeps getShotParameters()[2] calls out of the solver so the index
     * is only defined in one place.
     */
    private double shotTableTof(double dist) {
        return activeShotTable.getShotParameters(dist)[2];
    }

    public void setTarget(Supplier<Translation2d> target) {
        this.targetSupplier = target;
        this.activeShotTable = shotTable;
    }

    public void setPassing() {
        this.setTarget(kPassSupplier, adjustTargetForMovingShots, passingTable);
    }

    public void setTarget(Supplier<Translation2d> target, boolean movingShots) {
        this.setTarget(target);
        this.adjustTargetForMovingShots = movingShots;
    }

    public void setTarget(Supplier<Translation2d> target, boolean movingShots, ShotTable table) {
        this.setTarget(target, movingShots);
        this.activeShotTable = table;
        update();
    }

    public void update() {
        Pose2d robotPose = Robot.swerve.getPose();
        Translation2d robotXY = robotPose.getTranslation();

        ChassisSpeeds fieldSpeeds = Robot.swerve.getFieldRelativeSpeeds();
        Translation2d currentVel = new Translation2d(fieldSpeeds.vxMetersPerSecond,
                                                       fieldSpeeds.vyMetersPerSecond);

        Translation2d rawTarget = targetSupplier.get();

        if (adjustTargetForMovingShots) {
            double tof1 = shotTableTof(rawTarget.minus(robotXY).getNorm());
            Translation2d firstPassTarget = rawTarget.minus(currentVel.times(tof1));
            double tof2 = shotTableTof(firstPassTarget.minus(robotXY).getNorm());
            Translation2d secondPassTarget = rawTarget.minus(currentVel.times(tof2));

            compensatedTarget = secondPassTarget;
        } else {
            compensatedTarget = rawTarget;
        }

        double distance = compensatedTarget.minus(robotXY).getNorm();

        double[] shotParams  = activeShotTable.getShotParameters(distance);

        Logger.recordOutput("TargetDistance", distance);
        Logger.recordOutput("CompensatedTarget", new Pose2d(compensatedTarget, Rotation2d.kZero));

        this.targetHoodAngle = Rotations.of(shotParams[0]);
        this.targetFlywheelVelocity = RotationsPerSecond.of(shotParams[1]);
    }
}