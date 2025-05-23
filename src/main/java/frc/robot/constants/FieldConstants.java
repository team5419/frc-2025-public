package frc.robot.constants;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.google.gson.JsonSyntaxException;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Represents measurements of the field, all units in meters
 */
public class FieldConstants {
    public static double kFieldLength = Units.inchesToMeters(690.875);
    public static double kFieldWidth = Units.inchesToMeters(317.0);

    public static final Pose2d kFrontReefPose = new Pose2d(14.205, 4.127, Rotation2d.fromDegrees(180));
    public static final Rotation3d kSourceRotationLeft =
            new Rotation3d(new Quaternion(-0.8910065241883679, 0.0, 0.0, 0.45399049973954675));
    public static final Rotation3d kSourceRotationRight =
            new Rotation3d(new Quaternion(0.8910065241883679, 0.0, 0.0, 0.45399049973954675));

    public static class CoralTags {

        public static final List<Pose3d> kCoralTags = new ArrayList<>(); // list of coral tags on the reef

        public static final Translation2d kReefOffset = new Translation2d(0.456, 0.0); // So we are not in the reef
        // early: x: 0.443, y: 0.166, rotation: 3.136
        // late: x: 0.4357, y: -0.187, rotation: 3.139
        public static final Translation2d kCoralAlignOffset = new Translation2d(
                0.0,
                0.12
                        - (Units.inchesToMeters(2))
                        - Units.inchesToMeters(
                                1)); // .177 The horizontal offset between the tag and the left and right scoring
        // positions at the
        // reef

        public static final String jsonFilePath = Filesystem.getDeployDirectory() + "/2025-reefscape.json";

        static {
            try {
                JsonElement jsonElement = JsonParser.parseReader(new FileReader(jsonFilePath));
                JsonObject jsonObject = jsonElement.getAsJsonObject();
                JsonArray tagsArray = jsonObject.getAsJsonArray("tags");

                for (JsonElement tagElement : tagsArray) {
                    JsonObject tagObject = tagElement.getAsJsonObject();
                    int id = tagObject.get("ID").getAsInt();
                    if (id < 17 || id > 22) continue; // we only want the coral tags here!
                    JsonObject poseObject = tagObject.getAsJsonObject("pose");
                    JsonObject translationObject = poseObject.getAsJsonObject("translation");
                    double x = translationObject.get("x").getAsDouble();
                    double y = translationObject.get("y").getAsDouble();
                    double z = translationObject.get("z").getAsDouble();

                    JsonObject rotationObject = poseObject.getAsJsonObject("rotation");
                    JsonObject quaternionObject = rotationObject.getAsJsonObject("quaternion");
                    double qw = quaternionObject.get("W").getAsDouble();
                    double qx = quaternionObject.get("X").getAsDouble();
                    double qy = quaternionObject.get("Y").getAsDouble();
                    double qz = quaternionObject.get("Z").getAsDouble();

                    Quaternion quaternion = new Quaternion(qw, qx, qy, qz);

                    Rotation3d rotation = new Rotation3d(quaternion);

                    Pose3d pose = new Pose3d(new Translation3d(x, y, z), rotation);

                    kCoralTags.add(pose);
                }

                System.out.println("Loaded " + kCoralTags.size() + " tags from " + jsonFilePath);

            } catch (IOException | JsonSyntaxException e) {
                e.printStackTrace();
            }
        }
    }
}
