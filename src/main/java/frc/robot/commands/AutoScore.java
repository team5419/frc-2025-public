package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.commands.elevator.SafeElevate;
import frc.robot.commands.rollers.Outtake;
import frc.robot.commands.swerve.AutoAlignToCoral;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.roller.Roller.RollerGoal;

public class AutoScore extends SequentialCommandGroup {
    public AutoScore(RobotContainer robot, CommandXboxController driver) {

        Elevator elevator = robot.getElevator();
        Pivot pivot = robot.getPivot();

        addRequirements(elevator, pivot);

        addCommands(
                new InstantCommand(() -> robot.getRoller().setCurrentGoal(RollerGoal.GENTLE_INTAKE)),
                new ConditionalCommand(
                        new ScoreL1(robot),
                        new SequentialCommandGroup(
                                new ParallelRaceGroup(
                                        new AutoAlignToCoral(robot, driver),
                                        new SafeElevate(robot, () -> elevator.getDesiredLevel(), false)),
                                new ConditionalCommand(
                                        new SequentialCommandGroup(
                                                new WaitCommand(0.1),
                                                new Outtake(
                                                        robot, () -> elevator.getDesiredLevel() == ElevatorGoal.L4)),
                                        new InstantCommand(),
                                        (() -> RobotState.getInstance().isAutoAlignAtGoal())),
                                new InstantCommand(
                                        () -> RobotState.getInstance().setAutoAlignAtGoal(false))),
                        () -> elevator.getDesiredLevel() == ElevatorGoal.L1),
                new InstantCommand(() -> robot.getRoller().setCurrentGoal(RollerGoal.IDLE)));
    }
}
