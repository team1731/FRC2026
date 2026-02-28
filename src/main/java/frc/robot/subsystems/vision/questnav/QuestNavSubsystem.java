package frc.robot.subsystems.vision.questnav;

import frc.lib.frc1731.TriConsumer;
import frc.robot.subsystems.BaseSubsystem;
import gg.questnav.questnav.*;

import static frc.robot.subsystems.vision.questnav.VSLAMConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;

public class QuestNavSubsystem extends BaseSubsystem {
    private QuestNav questNav;

    private static boolean isConnected = false;
    private static boolean isTracking = false;

    private Timer poseResetTimer = new Timer();

    private Pose3d currentPose = new Pose3d();

    private static Matrix<N3, N1> QUESTNAV_STD_DEVS =
    VecBuilder.fill(
        0.02, // Trust down to 2cm in X direction
        0.02, // Trust down to 2cm in Y direction
        0.035 // Trust down to 2 degrees rotational
    );

    public QuestNavSubsystem(boolean enabled) {
        super(enabled);
        questNav = new QuestNav();
        resetPose(new Pose3d());
    }

    @Override
    public void periodicTelemetry() {
        questNav.commandPeriodic();
        isConnected = questNav.isConnected();
        isTracking = questNav.isTracking();

        logger.log("Is Connected", isConnected);
        logger.log("Is Tracking", isTracking);
        logger.log("Current Pose", currentPose);
    }

    public Pose3d getPose() {
        // Get the latest pose data frames from the Quest
        PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();

        if (poseFrames.length > 0) {
            // Get the most recent Quest pose
            Pose3d questPose = poseFrames[poseFrames.length - 1].questPose3d();

            // Transform by the mount pose to get your robot pose
            Pose3d robotPose = questPose.transformBy(kRobotToOculus.inverse());
            return robotPose;
        }
        return null;
    }

    public void resetPose(Pose3d robotPose) {
        // Transform the robot pose by the mount pose to get the corresponding Quest pose
        Pose3d questResetPose = robotPose.transformBy(kRobotToOculus);

        // Set the QuestNav pose to the calculated Quest pose
        questNav.setPose(questResetPose);
    }

    public boolean isConnected() {
        return isConnected;
    }

    public boolean isTracking() {
        return isTracking;
    }

    public void addVisionMeasurement(TriConsumer<Pose2d, Double, Matrix<N3, N1>> measurementConsumer) {
        poseResetTimer.start();

        // Get the latest pose data frames from the Quest
        PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

        // Loop over the pose data frames and send them to the pose estimator
        for (PoseFrame questFrame : questFrames) {
            // Make sure the Quest was tracking the pose for this frame
            if (questFrame.isTracking()) {
                // Get the pose of the Quest
                Pose3d questPose = questFrame.questPose3d();
                // Get timestamp for when the data was sent
                double timestamp = questFrame.dataTimestamp();

                // Transform by the mount pose to get your robot pose
                Pose3d robotPose = questPose.transformBy(kRobotToOculus.inverse());

                // Add the measurement to our estimator if 20 ms has passed since last update
                if (poseResetTimer.hasElapsed(0.2)) {
                    measurementConsumer.accept(robotPose.toPose2d(), timestamp, QUESTNAV_STD_DEVS);
                    poseResetTimer.reset();
                }
            }
        }
    }
}