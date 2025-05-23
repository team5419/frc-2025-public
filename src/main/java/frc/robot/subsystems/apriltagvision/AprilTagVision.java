package frc.robot.subsystems.apriltagvision;

import static frc.robot.subsystems.apriltagvision.AprilTagVisionConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.lib.VirtualSubsystem;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;
import lombok.Getter;
import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

public class AprilTagVision extends VirtualSubsystem {

    private AprilTagVisionIO[] ios;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private final String loggingRoot = "AprilTagVision/";

    @Getter
    private boolean isReefTagDetected = false;

    private RobotContainer robot;
    private Alert camerasDisconnected = new Alert("Both cameras disconnected!", AlertType.kError);

    private double lastTimestamp;
    private boolean shouldJump;
    private double jumpTimestamp;

    public AprilTagVision(RobotContainer robot, AprilTagVisionIO... io) {
        this.robot = robot;
        ios = io;
        try {
            aprilTagFieldLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/2025-reefscape.json");
        } catch (IOException e) {
            DriverStation.reportError("[VISION] [ERROR] Layout not found", true);
            System.exit(1); // If you want
        }
        if (AprilTagVisionConstants.kUsingVision) {
            for (AprilTagVisionIO visionIO : ios) {
                visionIO.addCamerasToEstimators(aprilTagFieldLayout);
            }
        }
    }

    @Override
    public void periodic() {
        Logger.recordOutput(loggingRoot + "Pure Odometry", robot.getSwerve().getPose());
        Logger.recordOutput(loggingRoot + "Reef tag detected", isReefTagDetected);
        if (!AprilTagVisionConstants.kUsingVision) return;
        for (AprilTagVisionIO io : ios) {
            Map<String, VisionUpdate> estimates = io.getEstimates(
                    RobotState.getInstance().isAligning(), robot.getSwerve().getPose());
            Logger.recordOutput(loggingRoot + "estimate size", estimates.size());
            for (int i = 0; i < estimates.values().size(); i++) {
                VisionUpdate vu = estimates.values().toArray(new VisionUpdate[i])[i];
                Logger.recordOutput(loggingRoot + "Estimated Pose" + i, vu.estimatedPose.toPose2d());
            }
            // previous standard deviation constants instead of using function to find them
            // double devX = 1.5; // m
            // double devY = 1.5; // m
            // double devRot = 3.5; // radF

            camerasDisconnected.set(!io.getIsConnected());
            RobotState.getInstance().setVisionConnected(io.getIsConnected());
            int i = 0;

            var filteredEstimates = filterEstimates(estimates);
            for (VisionUpdate estimate : filteredEstimates.values()) {
                var lastShouldJump = shouldJump;
                if (RobotState.getInstance().isAligning()
                        && estimate.cameraName == AprilTagVisionConstants.BACK_CAMERA_NAME) continue;

                shouldJump = Timer.getFPGATimestamp() - lastTimestamp > kMissingReadingsDelta;
                if (!lastShouldJump && shouldJump) jumpTimestamp = Timer.getFPGATimestamp();
                if (Timer.getFPGATimestamp() - jumpTimestamp < kStopHighTrust) shouldJump = true;

                double devXY = io.translationalStdDev(
                        estimate.targetsUsed,
                        shouldJump ? kHighTrustTranslationalStdDevMultiplier : kTranslationalStdDevMultiplier);
                double devRot = io.rotationalStdDev(
                        estimate.targetsUsed,
                        shouldJump ? kHighTrustRotationalStdDevMultiplier : kRotationalStdDevMultiplier);
                // add weighted estimate to swerve pose estimator
                robot.getSwerve()
                        .addVisionMeasurement(
                                estimate.getEstimatedPose().toPose2d(),
                                estimate.timestamp,
                                new Matrix<N3, N1>(new SimpleMatrix(new double[] {devXY, devXY, devRot})));

                lastTimestamp = Timer.getFPGATimestamp();
                Logger.recordOutput(loggingRoot + "Should Jump", shouldJump);

                Logger.recordOutput(loggingRoot + "Estimate With Filter " + i, estimate.estimatedPose.toPose2d());
                i++;
            }

            Logger.recordOutput(loggingRoot + "Timestamp delta", Timer.getFPGATimestamp() - lastTimestamp);
            Logger.recordOutput(loggingRoot + "Jump timestamp", jumpTimestamp);
        }
    }
    // direct removal of estimates that are inaccurate
    public Map<String, VisionUpdate> filterEstimates(Map<String, VisionUpdate> estimates) {
        Map<String, VisionUpdate> filtered = new HashMap<>();
        isReefTagDetected = false;
        if (estimates.isEmpty()) return filtered;
        for (String cameraName : estimates.keySet()) {
            VisionUpdate estimate = estimates.get(cameraName);
            double totalTags = estimate.targetsUsed.size();
            double totalDistance = 0;
            double totalAmbiguity = 0;
            double totalArea = 0;

            for (PhotonTrackedTarget target : estimate.targetsUsed) {
                totalAmbiguity += target.getPoseAmbiguity();
                totalDistance += target.getBestCameraToTarget().getTranslation().getNorm();
                totalArea += target.getArea();

                if ((target.getFiducialId() >= 6
                                && target.getFiducialId() <= 11
                                && DriverStation.getAlliance().get() == Alliance.Red)
                        || (target.getFiducialId() >= 17
                                && target.getFiducialId() <= 22
                                && DriverStation.getAlliance().get() == Alliance.Blue)) {
                    isReefTagDetected = true;
                }
            }

            double averageAmbiguity = totalAmbiguity / totalTags;
            double averageDistance = totalDistance / totalTags;
            double averageArea = totalArea / totalTags;

            if (!averageAmbiguityCheck(averageAmbiguity)) continue;

            if (!averageDistanceCheck(averageDistance)) continue;

            if (!averageAreaCheck(averageArea)) continue;

            if (!averageSpeedCheck()) continue;

            Logger.recordOutput(loggingRoot + "Rejecting Reason", "No Rejection, All checks passed!");

            filtered.put(cameraName, estimate);
        }
        return filtered;
    }

    public boolean averageSpeedCheck() {
        // if average speed too high we're cooked chat
        ChassisSpeeds speeds = robot.getSwerve().getChassisSpeeds();
        if (robot.getSwerve().getAbsoluteRobotVelocity() > AprilTagVisionConstants.kMaxVelocity
                || Math.abs(speeds.omegaRadiansPerSecond) > AprilTagVisionConstants.kMaxRotationalVelocity) {
            Logger.recordOutput(
                    loggingRoot + "Rejecting Reason",
                    "Rejecton on speed, Calculated speeds, VX: "
                            + speeds.vxMetersPerSecond + " VY:"
                            + speeds.vyMetersPerSecond + " Omega: " + speeds.omegaRadiansPerSecond);
            return false;
        }
        return true;
    }

    public boolean averageAmbiguityCheck(double averageAmbiguity) {
        // If average ambiguity is too high then reject
        boolean isAmbiguityTooHigh = averageAmbiguity >= AprilTagVisionConstants.kAmbiguityRejectionLimit;
        if (isAmbiguityTooHigh) {
            Logger.recordOutput(
                    loggingRoot + "Rejecting Reason",
                    "Rejecting on ambiguity, Calculated ambiguity is: " + averageAmbiguity);
            return false;
        }
        return true;
    }

    public boolean averageDistanceCheck(double averageDistance) {
        // If average distance if too high then reject
        if (averageDistance >= AprilTagVisionConstants.kDistanceRejectionLimit) {
            Logger.recordOutput(
                    loggingRoot + "Rejecting Reason", "Rejecting on distance, Calculated distance: " + averageDistance);
            return false;
        }
        return true;
    }

    public boolean averageAreaCheck(double averageArea) {
        // If average distance if too high then reject
        if (averageArea <= AprilTagVisionConstants.kAreaMinimum) {
            Logger.recordOutput(loggingRoot + "Rejecting Reason", "Rejecting on area, Calculated area: " + averageArea);
            return false;
        }
        return true;
    }
}
