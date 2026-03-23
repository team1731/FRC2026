package frc.lib.frc1731.hardware.camera.photonvision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.frc1731.TriConsumer;

public class PNPEstimationHelper {

    public static void updatePoseEstimations(PNPCamera camera, TriConsumer<Pose2d, Double, Matrix<N3, N1>> addVisionMeasurementConsumer) {
        if(camera == null) return;
        PhotonCamera photonCamera = camera.getCamera();
        PhotonPoseEstimator poseEstimator = camera.getPoseEstimator();
        if(photonCamera == null || poseEstimator == null) return;

        if(poseEstimator != null) {
            // Correct pose estimate with vision measurements
            try {
                var visionEst = poseEstimator.update(camera.getLatestResult());
                //isZoomCameraReadingValid = visionEst.isPresent();
                visionEst.ifPresent(
                    est -> {
                        var estPose = est.estimatedPose.toPose2d();
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = getEstimationStdDevs(photonCamera, estPose, poseEstimator);
                        //field2d.getObject("MyRobot" + camera.getName()).setPose(estPose);
                        SmartDashboard.putString("Vision pose", String.format("(%.2f, %.2f) %.2f",
                                estPose.getTranslation().getX(),
                                estPose.getTranslation().getY(),
                                estPose.getRotation().getDegrees()));
                        camera.setLastUpdatedTS(Timer.getFPGATimestamp());
                        camera.setEstimatedPose(est.estimatedPose.toPose2d());
                        addVisionMeasurementConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                    });
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    /**
     * The standard deviations of the estimated pose from
     * {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
     * SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public static Matrix<N3, N1> getEstimationStdDevs(PhotonCamera camera, Pose2d estimatedPose, PhotonPoseEstimator photonEstimator) {
        var estStdDevs = VecBuilder.fill(2, 2, 8);
        var targets = camera.getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty())
                continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }

        if (numTags == 0) {
            return estStdDevs;
        }

        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) {
            estStdDevs = VecBuilder.fill(0.05, 0.05, 1);
        }

        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }
}
