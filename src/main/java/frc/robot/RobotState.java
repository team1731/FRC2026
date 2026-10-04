package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Objects;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.measure.*;
import frc.lib.frc1678.mechviz.RobotVisualization;
import frc.lib.frc1678.mechviz.RobotVisualizer;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.hopper.HopperConstants;
import frc.robot.subsystems.intakedeploy.IntakeDeployConstants;

/**
 * Shared, hardware-free swerve cache published on the robot scheduler thread.
 *
 * <p>Readers never instantiate subsystems or poll hardware. Values are the last published
 * sample, not a guarantee that a subsystem is active. An inactive subsystem retains its last
 * sample. Use the drive timestamp when freshness matters.
 */
public final class RobotState {
    private RobotState() {}

    private static Pose2d robotPose = new Pose2d();
    private static Angle hoodAngle = HoodConstants.kHomeAngle;
    private static Angle intakeDeployAngle = IntakeDeployConstants.kHomeAngle;
    private static Distance squeezerHeight = HopperConstants.kHomeHeight;

    /** Last field-relative pose; origin until swerve publishes its first sample. */
    public static Pose2d getRobotPose() { 
        return robotPose; 
    }

    public static Angle getHoodAngle() {
        return hoodAngle;
    }

    public static Angle getIntakeDeployAngle() {
        return intakeDeployAngle;
    }

    public static Distance getSqueezerHeight() {
        return squeezerHeight;
    }

    /** Called by swerve with its existing cached state, on the scheduler thread. */
    public static void updateDrive(Pose2d pose) {
        Objects.requireNonNull(pose);
        robotPose = pose;
    }

    public static void updateHood(Angle angle) {
        Objects.requireNonNull(angle);
        hoodAngle = angle;
    }

    public static void updateIntake(Angle angle) {
        Objects.requireNonNull(angle);
        intakeDeployAngle = angle;
    }

    public static void updateHopper(Distance height) {
        Objects.requireNonNull(height);
        squeezerHeight = height;
    }

    public static RobotVisualization createMechViz() {
        return RobotVisualizer.builder(RobotConstants.kRobotName)
            .withRobotPose(RobotState::getRobotPose)
            .withArm("IntakeDeploy", new Pose3d(), () -> RobotState.getIntakeDeployAngle().in(Radians))
            .withArm("Hood", new Pose3d(), () -> RobotState.getHoodAngle().in(Radians))
            .withElevator("Squeezer", new Pose3d(), () -> RobotState.getSqueezerHeight().in(Meters))
            .build();
    }
}