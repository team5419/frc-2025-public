package frc.robot.subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.ArrayList;
import java.util.List;
import lombok.Builder;
import lombok.Getter;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

@Builder
public class VisionUpdate implements LoggableInputs {
    @Getter
    String cameraName;

    @Getter
    Pose3d estimatedPose;

    @Getter
    double devXY;

    @Getter
    double devRot;

    @Getter
    double timestamp;

    @Getter
    List<PhotonTrackedTarget> targetsUsed;

    @Getter
    PoseStrategy strategy;

    public VisionUpdate(
            String cameraName,
            Pose3d estimatedPose,
            double devXY,
            double devRot,
            double timestamp,
            List<PhotonTrackedTarget> targetsUsed,
            PoseStrategy strategy) {
        this.cameraName = cameraName;
        this.estimatedPose = estimatedPose;
        this.devXY = devXY;
        this.devRot = devRot;
        this.timestamp = timestamp;
        this.targetsUsed = targetsUsed;
        this.strategy = strategy;
    }

    public VisionUpdate() {
        this.cameraName = "";
        this.estimatedPose = new Pose3d();
        this.devXY = 0;
        this.devRot = 0;
        this.timestamp = -1;
        this.targetsUsed = new ArrayList<>();
        this.strategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    }

    @Override
    public void toLog(LogTable table) {
        table.put("Camera Name", cameraName);
        table.put("Estimated Pose", estimatedPose);
        table.put("Translation StdDev", devXY);
        table.put("Rotation StdDev", devRot);
        table.put("Timestamp", timestamp);
        table.put("Strategy", strategy);
    }

    @Override
    public void fromLog(LogTable table) {
        table.get("Camera Name", "");
        table.get("Estimated Pose", new Pose3d());
        table.get("Translation StdDev", 0);
        table.get("Rotation StdDev", 0);
        table.get("Timestamp", -1);
        table.get("Strategy", PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
    }
}
