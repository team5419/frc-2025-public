package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.commands.elevator.OptionalElevate;
import frc.robot.commands.elevator.SafeElevate;
import frc.robot.commands.pivot.PivotToPos;
import frc.robot.lib.util.GeomUtil;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.pivot.Pivot.PivotGoal;

public class DisplaceAlgae extends SequentialCommandGroup {
    private static double kOffsetIntoReef = -0.10;

    public DisplaceAlgae(RobotContainer robot, CommandXboxController driver) {
        var swerve = robot.getSwerve();
        var elevator = robot.getElevator();
        var pivot = robot.getPivot();
        addRequirements(swerve, elevator, pivot);

        addCommands(
                new ParallelCommandGroup(
                        new DriveToPos(
                                robot,
                                driver,
                                () -> swerve.getBestReefTagNoOffset()
                                        .plus(GeomUtil.toTransform2d(new Translation2d(0.85, kOffsetIntoReef)))
                                        .plus(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180))),
                                () -> false,
                                () -> false),
                        new SequentialCommandGroup(
                                new OptionalElevate(robot, () -> ElevatorGoal.STOW),
                                new PivotToPos(robot, () -> PivotGoal.SHOOT))),
                new DriveToPos(
                        robot,
                        driver,
                        () -> swerve.getBestReefTagNoOffset()
                                .plus(GeomUtil.toTransform2d(new Translation2d(0.4, kOffsetIntoReef)))
                                .plus(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180))),
                        () -> false,
                        () -> false),
                new ParallelRaceGroup(
                        new SafeElevate(robot, () -> ElevatorGoal.L4),
                        new SequentialCommandGroup(
                                new WaitCommand(0.4),
                                Commands.run(() -> swerve.runVelocity(new ChassisSpeeds(
                                        -1, 0, RobotState.getInstance().isEarly() ? -1 : 1))))),
                new InstantCommand(() -> RobotState.getInstance().setDisplacing(false)),
                new InstantCommand(
                        () -> RobotState.getInstance().setDisplacing(false))); // We might not need to move backwards
    }
}
