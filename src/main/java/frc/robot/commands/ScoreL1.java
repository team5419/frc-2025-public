package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.elevator.SafeElevate;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.pivot.Pivot.PivotGoal;
import frc.robot.subsystems.roller.Roller.RollerGoal;

public class ScoreL1 extends SequentialCommandGroup {

    public ScoreL1(RobotContainer robot) {

        var pivot = robot.getPivot();

        addRequirements(pivot, robot.getElevator(), robot.getRoller());
        addCommands(
                new SafeElevate(robot, () -> ElevatorGoal.L1),
                new InstantCommand(() -> pivot.setCurrentGoal(PivotGoal.L1)),
                new WaitCommand(0.075),
                new InstantCommand(() -> robot.getRoller().setCurrentGoal(RollerGoal.L1_OUTTAKE)),
                new ParallelRaceGroup(new WaitCommand(0.3), new WaitUntilCommand(() -> !robot.getRoller()
                        .hasCoral())),
                new WaitCommand(0.1),
                new InstantCommand(() -> robot.getRoller().stop()),
                new WaitCommand(0.5),
                new InstantCommand(() -> pivot.setCurrentGoal(PivotGoal.HIGH)));
    }
}
