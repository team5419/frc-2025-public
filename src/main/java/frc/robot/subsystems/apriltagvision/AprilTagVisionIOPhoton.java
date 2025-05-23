package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagVisionIOPhoton implements AprilTagVisionIO {

    private List<PhotonCamera> cameras = new ArrayList<>();
    private List<PhotonPoseEstimator> multiTagPhotonEstimators = new ArrayList<>();
    private List<PhotonPoseEstimator> reefTagPhotonEstimators = new ArrayList<>();
    private final String loggingRoot = "AprilTagVision/";
    private boolean isConnected;
    private boolean lastIsAutoAligning = false;

    public AprilTagVisionIOPhoton() {}

    @Override
    public void addCamerasToEstimators(AprilTagFieldLayout aprilTagFieldLayout) {
        for (int i = 0; i < AprilTagVisionConstants.kCameraQuantity; i++) {
            cameras.add(new PhotonCamera(AprilTagVisionConstants.kCameraNames[i]));

            // configure estimator for general pose estimation
            multiTagPhotonEstimators.add(new PhotonPoseEstimator(
                    aprilTagFieldLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    AprilTagVisionConstants.kCameraTransform[i]));
            multiTagPhotonEstimators.get(i).setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

            // configure estimator for reef pose estimation (autoalign)
            reefTagPhotonEstimators.add(new PhotonPoseEstimator(
                    aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, AprilTagVisionConstants.kCameraTransform[i]));
        }
    }

    @Override
    public Map<String, VisionUpdate> getEstimates(boolean isAutoAligning, Pose2d lastPose) {
        // if we start or stop autoaligning switched is true
        boolean switched = lastIsAutoAligning != isAutoAligning;
        lastIsAutoAligning = isAutoAligning;
        Map<String, VisionUpdate> estimates = new HashMap<>();
        boolean connected = true;
        for (int i = 0; i < cameras.size(); i++) {
            PhotonPoseEstimator reefEstimator = reefTagPhotonEstimators.get(i);
            PhotonPoseEstimator multiTagEstimator = multiTagPhotonEstimators.get(i);
            PhotonCamera camera = cameras.get(i);
            Logger.recordOutput(loggingRoot + "Camera is active " + camera.getName(), camera.isConnected());
            connected &= camera.isConnected(); // if all the cameras are connected, connected = true
            var results = camera.getAllUnreadResults();
            Logger.recordOutput(loggingRoot + "Estimates size ", results.size());
            for (PhotonPipelineResult result : results) {
                // if we switched, set the reference pose of the estimator set to be used so we avoid lag
                if (switched) {
                    (isAutoAligning ? reefEstimator : multiTagEstimator).setReferencePose(lastPose);
                }
                // reefEstimator has LOWEST_AMBIGUITY strategy, so only looks at the apriltag on the reef while
                // autoaligning, multiTagEstimator will use all tags and is better for general odometry updates
                Optional<EstimatedRobotPose> maybeEstimation =
                        (isAutoAligning ? reefEstimator : multiTagEstimator).update(result);

                if (maybeEstimation.isEmpty()) continue;
                var eRobotPose = maybeEstimation.get();
                // "stddevs" means how good our estimate is, and how much we should weigh it
                var visionUpdate = new VisionUpdate(
                        camera.getName(),
                        eRobotPose.estimatedPose,
                        translationalStdDev(
                                eRobotPose.targetsUsed, AprilTagVisionConstants.kTranslationalStdDevMultiplier),
                        rotationalStdDev(eRobotPose.targetsUsed, AprilTagVisionConstants.kRotationalStdDevMultiplier),
                        eRobotPose.timestampSeconds,
                        eRobotPose.targetsUsed,
                        eRobotPose.strategy);
                estimates.put(camera.getName(), visionUpdate);
            }
        }
        isConnected = connected;
        return estimates;
    }
    // taken from mechanical advantage with slight tuning
    @Override
    public double translationalStdDev(List<PhotonTrackedTarget> targetsUsed, double std_dev_multiplier) {
        double lowest_dist = Double.POSITIVE_INFINITY;
        double total_dist = 0;
        double numTargets = targetsUsed.size();
        for (PhotonTrackedTarget target : targetsUsed) {
            var distance = target.getBestCameraToTarget().getTranslation().getNorm();
            if (distance < lowest_dist) {
                lowest_dist = distance;
            }
            total_dist += distance;
        }
        double xyStdDev = std_dev_multiplier
                * ((Math.pow(lowest_dist, 2.0))
                        + (AprilTagVisionConstants.kAverageDistanceStdDevMultiplier
                                * Math.pow(total_dist / numTargets, 2.0)))
                / numTargets;
        xyStdDev = Math.max(AprilTagVisionConstants.kMinimumVisionStdDev, xyStdDev);
        Logger.recordOutput(loggingRoot + "xystdv", xyStdDev);
        return xyStdDev;
    }

    @Override
    public double rotationalStdDev(List<PhotonTrackedTarget> estimate, double std_dev_multiplier) {
        return std_dev_multiplier;
    }

    @Override
    public boolean getIsConnected() {
        return isConnected;
    }
}
