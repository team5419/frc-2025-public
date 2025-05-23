package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.Pivot.PivotGoal;

public class WaitForPivotSafe extends SequentialCommandGroup {
    public WaitForPivotSafe(Pivot pivot) {
        addRequirements(pivot);

        addCommands(
                new InstantCommand(() -> pivot.setCurrentGoal(
                        RobotState.getInstance().getDesiredElePos() == ElevatorGoal.L4
                                ? PivotGoal.L4
                                : PivotGoal.SHOOT)),
                new WaitUntilCommand(() -> pivot.isSafe() && pivot.getCurrentGoal() != PivotGoal.HIGH));
    }
}
