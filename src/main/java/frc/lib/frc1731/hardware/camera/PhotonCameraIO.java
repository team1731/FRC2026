package frc.lib.frc1731.hardware.camera;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.lib.frc6328.FieldConstants;
import frc.robot.Robot;
import frc.robot.subsystems.vision.VisionConstants;

/**
 * AprilTag IO implementation backed by PhotonVision and PhotonLib.
 *
 * <p>The class caches the latest usable pose estimate during {@link #periodic(double)} so
 * {@code VisionHandler} can consume it through the shared {@link AprilTagIO} interface.
 */
public class PhotonCameraIO implements AprilTagIO {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private final String name;

    private Matrix<N3, N1> curStdDevs = VisionConstants.kPhotonVisionSingleTagStdev;
    private Pose2d latestPose = null;
    private double latestTimestamp = 0.0;

    // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    /**
     * Creates a PhotonVision camera IO provider with a fixed robot-to-camera transform.
     *
     * @param name PhotonVision camera name
     * @param robotToCam transform from robot origin to camera
     */
    public PhotonCameraIO(String name, Transform3d robotToCam) {
        this.name = name;
        this.camera = new PhotonCamera(name);
        AprilTagFieldLayout layout = FieldConstants.AprilTagLayoutType.OFFICIAL.getLayout();
        photonEstimator = new PhotonPoseEstimator(layout, robotToCam);

        // ----- Simulation
        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim.addAprilTags(layout);
            // Create simulated camera properties. These can be set to mimic your actual camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
            // targets.
            cameraSim = new PhotonCameraSim(camera, cameraProp);
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(cameraSim, robotToCam);

            cameraSim.enableDrawWireframe(true);
        }
    }

    /**
     * Returns the configured PhotonVision camera name.
     */
    @Override
    public String getName() {
        return this.name;
    }

    /**
     * Reports PhotonVision camera connection state.
     */
    @Override
    public boolean isConnected() {
        return camera.isConnected();
    }

    /**
     * Returns dynamic measurement standard deviations calculated from the latest tag frame.
     */
    @Override
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    /**
     * Returns the timestamp of the latest accepted PhotonVision estimate.
     */
    @Override
    public double getTimestamp() {
        return latestTimestamp;
    }

    /**
     * Returns the latest accepted PhotonVision field pose estimate.
     */
    @Override
    public Pose2d getEstimatedPose() {
        return latestPose;
    }

    /**
     * Resets simulated camera pose history.
     */
    @Override
    public void resetSimPose(Pose2d robotPose) {
        if (Robot.isSimulation()) visionSim.resetRobotPose(robotPose);
    }

    /**
     * Polls unread PhotonVision frames and caches the newest pose estimate.
     */
    @Override
    public void periodic(double yaw) {
        latestPose = null;

        for (var result : camera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
            }
            updateEstimationStdDevs(visionEst, result.getTargets());

            if (Robot.isSimulation()) {
                visionEst.ifPresentOrElse(
                        est ->
                                getSimDebugField()
                                        .getObject("VisionEstimation")
                                        .setPose(est.estimatedPose.toPose2d()),
                        () -> {
                            getSimDebugField().getObject("VisionEstimation").setPoses();
                        });
            }

            visionEst.ifPresent(est -> {
                latestPose = est.estimatedPose.toPose2d();
                latestTimestamp = est.timestampSeconds;
            });
        }
    }

    /**
     * Updates the simulated PhotonVision scene from the robot pose.
     */
    @Override
    public void updateSimulation(Pose2d robotPose) {
        if (Robot.isSimulation() && visionSim != null) {
            visionSim.update(robotPose);
        }
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = VisionConstants.kPhotonVisionSingleTagStdev;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VisionConstants.kPhotonVisionSingleTagStdev;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = VisionConstants.kPhotonVisionSingleTagStdev;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = VisionConstants.kPhotonVisionMultiTagStdev;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    /** Returns the simulation debug field for visualizing robot and tag estimates. */
    private Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }
}
