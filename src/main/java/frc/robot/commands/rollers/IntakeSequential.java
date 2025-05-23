package frc.robot.commands.rollers;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.elevator.SafeElevate;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.Pivot.PivotGoal;
import frc.robot.subsystems.roller.Roller;

public class IntakeSequential extends SequentialCommandGroup {
    private final Roller roller;
    private final Pivot pivot;
    private final Elevator elevator;

    public IntakeSequential(RobotContainer robot, ElevatorGoal goal) {
        roller = robot.getRoller();
        pivot = robot.getPivot();
        elevator = robot.getElevator();
        addRequirements(roller, elevator, pivot);

        addCommands(
                new SafeElevate(robot, () -> goal),
                new Intake(robot, () -> goal == ElevatorGoal.INTAKE_FAR ? PivotGoal.FAR_INTAKE : PivotGoal.HIGH));
    }
}
