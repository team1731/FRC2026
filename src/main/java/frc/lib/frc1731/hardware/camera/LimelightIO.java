package frc.lib.frc1731.hardware.camera;


import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.frc6328.FieldConstants;
import frc.lib.frc1731.hardware.camera.LimelightHelpers.PoseEstimate;
import frc.robot.Robot;

/**
 * AprilTag IO implementation backed by a real Limelight on robot and a PhotonVision sim camera in
 * simulation.
 *
 * <p>Real mode reads LimelightHelpers estimates. Simulation mode uses PhotonVision simulation as an
 * approximation of a Limelight-style AprilTag camera so the rest of the robot can exercise the same
 * {@link AprilTagIO} path.
 */
public class LimelightIO implements AprilTagIO {
    private final String name;
    private Optional<PoseEstimate> estimate = Optional.empty();
    private Optional<EstimatedRobotPose> simEstimate = Optional.empty();
    private double heartbeat = 0;
    private int count = 0;
    private boolean megaTag2 = false;

    private PhotonCamera simCamera;
    private PhotonPoseEstimator simEstimator;
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    /**
     * Creates a Limelight IO provider with a fixed robot-to-camera transform.
     *
     * @param name NetworkTables name of the Limelight
     * @param robotToLimelight transform from robot origin to the Limelight
     */
    public LimelightIO(String name, Transform3d robotToLimelight) {
        this.name = name;
        LimelightHelpers.setCameraPose_RobotSpace(
            name, 
            robotToLimelight.getX(), 
            robotToLimelight.getY(), 
            robotToLimelight.getZ(), 
            Math.toDegrees(robotToLimelight.getRotation().getX()), 
            Math.toDegrees(robotToLimelight.getRotation().getY()), 
            Math.toDegrees(robotToLimelight.getRotation().getZ())
        );

        if (Robot.isSimulation()) {
            AprilTagFieldLayout layout = FieldConstants.AprilTagLayoutType.OFFICIAL.getLayout();
            simCamera = new PhotonCamera(name + "-sim");
            simEstimator = new PhotonPoseEstimator(layout, robotToLimelight);
            visionSim = new VisionSystemSim(name);
            visionSim.addAprilTags(layout);

            SimCameraProperties cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);

            cameraSim = new PhotonCameraSim(simCamera, cameraProp);
            visionSim.addCamera(cameraSim, robotToLimelight);
            cameraSim.enableDrawWireframe(true);
        }
    }

    /**
     * Returns the configured Limelight name.
     */
    @Override
    public String getName() {
        return this.name;
    }

    /**
     * Reports connection health from simulated camera state or real Limelight heartbeat.
     */
    @Override
    public boolean isConnected() {
        if (Robot.isSimulation()) {
            return simCamera != null && simCamera.isConnected();
        }

        return this.count < 50; // 1 second of disconnection results in the assumption that the limelight disconnected
    }

    /**
     * Returns distance- and tag-count-based deviations for the cached estimate.
     */
    @Override
    public Matrix<N3, N1> getEstimationStdDevs() {
        double distance = getTagDistance();
        int tagCount = Robot.isSimulation()
            ? simEstimate.map(est -> est.targetsUsed.size()).orElse(0)
            : estimate.map(est -> est.tagCount).orElse(0);
        return calculateStdDevs(distance, tagCount, megaTag2);
    }

    /** Calculates measurement deviations from the observed tag geometry and solver mode. */
    public static Matrix<N3, N1> calculateStdDevs(double distance, int tagCount, boolean megaTag2) {
        if (!Double.isFinite(distance) || distance <= 0 || tagCount <= 0) return null;
        double scale = distance * distance / tagCount;
        double position = (megaTag2 ? 0.01 : 0.02) * scale;
        return VecBuilder.fill(position, position,
            megaTag2 ? Double.POSITIVE_INFINITY : 0.06 * scale);
    }

    /**
     * Returns the latest real or simulated estimate timestamp.
     */
    @Override
    public double getTimestamp() {
        if (Robot.isSimulation()) {
            return simEstimate.map(est -> est.timestampSeconds).orElse(0.0);
        }

        return estimate.map(est -> est.timestampSeconds).orElse(0.0);
    }

    /**
     * Returns the latest real or simulated Limelight pose estimate.
     */
    @Override
    public Pose2d getEstimatedPose() {
        if (Robot.isSimulation()) {
            return simEstimate.map(est -> est.estimatedPose.toPose2d()).orElse(null);
        }

        if (estimate.isEmpty() || estimate.get().tagCount == 0) {
            return null;
        }

        return estimate.get().pose;
    }

    @Override
    public double getTagDistance() {
        if (Robot.isSimulation()) {
            return simEstimate.map(est -> est.targetsUsed.stream()
                .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
                .average().orElse(Double.NaN)).orElse(Double.NaN);
        }
        return estimate.map(est -> est.avgTagDist).orElse(Double.NaN);
    }
    
    /**
     * Resets simulated Limelight pose history.
     */
    @Override
    public void resetSimPose(Pose2d robotPose) {
        if (Robot.isSimulation() && visionSim != null) {
            visionSim.resetRobotPose(robotPose);
        }
    }

    /**
     * Polls the real Limelight or the simulated camera estimate.
     */
    @Override
    public void periodic(double yaw, double yawRate) {
        megaTag2 = DriverStation.isEnabled();
        if (Robot.isSimulation()) {
            updateSimEstimate();
            return;
        }

        LimelightHelpers.SetRobotOrientation(name, yaw, yawRate, 0, 0, 0, 0);
        if (megaTag2) {
            this.estimate = Optional.ofNullable(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name));
        } else {
            this.estimate = Optional.ofNullable(LimelightHelpers.getBotPoseEstimate_wpiBlue(name));
        }

        double curHeartbeat = LimelightHelpers.getHeartbeat(name);
        if (curHeartbeat == heartbeat) {
            count++;
        } else {
            count = 0;
        }

        heartbeat = LimelightHelpers.getHeartbeat(name);
    }

    /**
     * Updates the simulated Limelight scene from the robot pose.
     */
    @Override
    public void updateSimulation(Pose2d robotPose) {
        if (Robot.isSimulation() && visionSim != null) {
            visionSim.update(robotPose);
        }
    }

    /**
     * Reads all simulated camera frames and caches the newest estimate.
     */
    private void updateSimEstimate() {
        simEstimate = Optional.empty();

        if (simCamera == null || simEstimator == null) {
            return;
        }

        for (var result : simCamera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> visionEst = simEstimator.estimateCoprocMultiTagPose(result);
            if (visionEst.isEmpty()) {
                visionEst = simEstimator.estimateLowestAmbiguityPose(result);
            }

            simEstimate = visionEst;

            if (visionSim != null) {
                visionEst.ifPresentOrElse(
                    est -> getSimDebugField().getObject("VisionEstimation").setPose(est.estimatedPose.toPose2d()),
                    () -> getSimDebugField().getObject("VisionEstimation").setPoses()
                );
            }
        }
    }

    /**
     * Returns the simulation debug field for this camera.
     */
    private Field2d getSimDebugField() {
        if (Robot.isSimulation() && visionSim != null) {
            return visionSim.getDebugField();
        }
        return null;
    }
}
