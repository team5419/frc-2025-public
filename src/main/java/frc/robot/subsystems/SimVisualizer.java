package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotContainer;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.*;

public class SimVisualizer {

    private static final double MECHANISM_WIDTH = Units.feetToMeters(1);
    private static final double MECHANISM_HEIGHT = Units.feetToMeters(30);
    private static final Color8Bit MECHANISM_COLOR = new Color8Bit(Color.kBlack);
    private static final Translation3d MECHANISM_ORIGIN = new Translation3d(.1, .1, 0.4);

    private static final ArrayList<Double> elevatorHeights =
            new ArrayList<>(List.of(Units.feetToMeters(0), Units.feetToMeters(2), Units.feetToMeters(6)));

    private final LoggedMechanism2d mechanism2d;
    private final LoggedMechanismLigament2d elevatorMeasured;
    private final LoggedMechanismLigament2d elevatorTarget;
    private final LoggedMechanismLigament2d pivotMeasured;
    private final LoggedMechanismLigament2d pivotTarget;

    private final RobotContainer robot;

    public SimVisualizer(RobotContainer robot) {
        this.robot = robot;

        mechanism2d = new LoggedMechanism2d(MECHANISM_WIDTH, MECHANISM_HEIGHT, MECHANISM_COLOR);
        LoggedMechanismRoot2d root =
                mechanism2d.getRoot("Superstructure" + " Root", MECHANISM_ORIGIN.getX(), MECHANISM_ORIGIN.getY());

        elevatorMeasured = createElevatorLigament(root, " Measured", Color.kFirstBlue);
        elevatorTarget = createElevatorLigament(root, " Target", Color.kGreen);

        pivotMeasured = createPivotLigament(elevatorMeasured, " Measured", Color.kFirstRed);
        pivotTarget = createPivotLigament(elevatorTarget, " Target", Color.kGreen);
    }

    public void update() {
        updateMechanism2d();
        logMechanism3dPoses();
    }

    private void updateMechanism2d() {
        elevatorMeasured.setLength(
                Math.max(0.1, robot.getElevator().getPosition())); // 0.1 to stop it from dissappearing

        elevatorTarget.setLength(Math.max(
                0.1,
                Units.feetToMeters(
                        robot.getElevator().getCurrentGoal().getEleHeight().getAsDouble())));

        pivotMeasured.setAngle(robot.getPivot().getMotorAngle() - 90);
        pivotMeasured.setColor(
                robot.getRoller().hasCoral() ? new Color8Bit(Color.kWhite) : new Color8Bit(Color.kFirstRed));
        pivotTarget.setAngle(robot.getPivot().getCurrentGoal().getAngle().getAsDouble() - 90);
        pivotTarget.setLineWeight(4);

        Logger.recordOutput("Superstructure/2D", mechanism2d);
    }

    private void logMechanism3dPoses() {

        Pose3d robotPose = new Pose3d(robot.getSwerve().getPose());
        Pose3d rootPose = calculateRootPose(robotPose);
        Pose3d[] elevatorMeasuredPose =
                calculateElevatorPoses(rootPose, elevatorMeasured, robotPose.getRotation(), elevatorHeights);
        Pose3d elevatorTargetPose = calculateElevatorPose(rootPose, elevatorTarget);
        Pose3d endEffectorMeasuredPose = calculateEndEffectorPose(elevatorMeasuredPose[2], pivotMeasured);
        Pose3d endEffectorTargetPose = calculateEndEffectorPose(elevatorTargetPose, pivotTarget);

        Pose3d[] componentPosesMeasured = {
            elevatorMeasuredPose[0], elevatorMeasuredPose[1], elevatorMeasuredPose[2], endEffectorMeasuredPose
        };

        Logger.recordOutput("Superstructure/3D/ElevatorTargetPose", elevatorTargetPose);
        Logger.recordOutput("Superstructure/3D/EndEffectorMeasuredPose", endEffectorMeasuredPose);
        Logger.recordOutput("Superstructure/3D/EndEffectorTargetPose", endEffectorTargetPose);
        Logger.recordOutput(
                "Superstructure/3D/CoralInEndEffector",
                robot.getRoller().hasCoral() ? endEffectorTargetPose : new Pose3d());

        Logger.recordOutput("Superstructure/3D/ElevatorMeasuredPose", elevatorMeasuredPose);

        Logger.recordOutput("Superstructure/3D/ComponenetPoses", componentPosesMeasured);
    }

    private LoggedMechanismLigament2d createElevatorLigament(LoggedMechanismRoot2d root, String suffix, Color color) {
        return root.append(new LoggedMechanismLigament2d(
                "Superstructure" + " Elevator" + suffix, Units.inchesToMeters(0.0), 90, 6.0, new Color8Bit(color)));
    }

    private LoggedMechanismLigament2d createPivotLigament(
            LoggedMechanismLigament2d parent, String suffix, Color color) {
        return parent.append(new LoggedMechanismLigament2d(
                "Superstructure" + " Pivot" + suffix, Units.inchesToMeters(10.0), 0.0, 8.0, new Color8Bit(color)));
    }

    private Pose3d calculateRootPose(Pose3d robotPose) {
        return new Pose3d(
                MECHANISM_ORIGIN.getX() + robotPose.getX(),
                robotPose.getY(),
                MECHANISM_ORIGIN.getZ(),
                robotPose.getRotation());
    }

    private Pose3d[] calculateElevatorPoses(
            Pose3d rootPose,
            LoggedMechanismLigament2d elevator,
            Rotation3d robotRotation,
            ArrayList<Double> elevatorHeights) {

        ArrayList<Pose3d> elePoses = new ArrayList<>();

        for (double height : elevatorHeights) {

            elePoses.add(new Pose3d(
                    rootPose.getX(),
                    rootPose.getY(),
                    Math.min(rootPose.getZ() + elevator.getLength(), height),
                    robotRotation));
        }

        return elePoses.toArray(new Pose3d[elevatorHeights.size() - 2]); // better end effector
    }

    private Pose3d calculateElevatorPose(Pose3d rootPose, LoggedMechanismLigament2d elevator) {

        return new Pose3d(rootPose.getX(), rootPose.getY(), rootPose.getZ() + elevator.getLength(), new Rotation3d());
    }

    private Pose3d calculateEndEffectorPose(Pose3d elevatorPose, LoggedMechanismLigament2d pivot) {

        return new Pose3d(
                elevatorPose.getX(),
                elevatorPose.getY(),
                elevatorPose.getZ() + 0.1,
                new Rotation3d(0, -Units.degreesToRadians(pivot.getAngle()) + 90, 0));
    }
}
