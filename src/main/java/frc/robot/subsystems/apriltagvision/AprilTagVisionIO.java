// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.photonvision.targeting.PhotonTrackedTarget;

public interface AprilTagVisionIO {

    public default void addCamerasToEstimators(AprilTagFieldLayout layout) {}

    public default Map<String, VisionUpdate> getEstimates(boolean isAutoAligning, Pose2d lastPose) {
        return new HashMap<>();
    }

    public default double translationalStdDev(List<PhotonTrackedTarget> targetsUsed, double i) {
        return 0.0;
    }

    public default double rotationalStdDev(List<PhotonTrackedTarget> targetsUsed, double i) {
        return 0.0;
    }

    public default boolean getIsConnected() {
        return false;
    }
}
