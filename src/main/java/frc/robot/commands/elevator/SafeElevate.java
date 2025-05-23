package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.pivot.PivotToPos;
import frc.robot.commands.pivot.WaitForPivotSafe;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.pivot.Pivot.PivotGoal;
import frc.robot.subsystems.roller.Roller.RollerGoal;
import java.util.function.Supplier;

public class SafeElevate extends SequentialCommandGroup {

    public SafeElevate(RobotContainer robot, Supplier<ElevatorGoal> elevatorGoal) {
        this(robot, elevatorGoal, true);
    }

    public SafeElevate(RobotContainer robot, Supplier<ElevatorGoal> elevatorGoal, boolean shouldAutoExit) {
        var elevator = robot.getElevator();
        var pivot = robot.getPivot();
        var roller = robot.getRoller();

        addRequirements(elevator, pivot);

        addCommands(
                new ConditionalCommand(
                        new ParallelCommandGroup(
                                new WaitForPivotSafe(pivot),
                                new InstantCommand(() -> roller.setCurrentGoal(RollerGoal.GENTLE_INTAKE))),
                        new InstantCommand(),
                        () -> elevator.willCrossBar(elevatorGoal.get())),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new ElevateToPosition(elevator, roller, elevatorGoal),
                                new ConditionalCommand(
                                        new PivotToPos(
                                                robot,
                                                () -> elevator.getDesiredLevel() == ElevatorGoal.L4
                                                        ? PivotGoal.L4
                                                        : PivotGoal.SHOOT),
                                        new InstantCommand(),
                                        () -> elevator.isGoalScoring())),
                        new RunCommand(
                                () -> {
                                    if (elevator.getCurrentGoal() != elevatorGoal.get()) {
                                        elevator.setCurrentGoal(elevatorGoal.get());
                                        if (elevator.isGoalHigh(elevatorGoal.get()))
                                            roller.setCurrentGoal(RollerGoal.GENTLE_INTAKE);
                                    }
                                    var desiredPivotGoal = elevator.getDesiredLevel() == ElevatorGoal.L4
                                            ? PivotGoal.L4
                                            : PivotGoal.SHOOT;
                                    if (pivot.getCurrentGoal() != desiredPivotGoal)
                                        pivot.setCurrentGoal(desiredPivotGoal);
                                },
                                elevator),
                        () -> shouldAutoExit),
                new ConditionalCommand(
                        new InstantCommand(() -> roller.setCurrentGoal(RollerGoal.IDLE)),
                        new InstantCommand(),
                        () -> roller.getCurrentGoal() == RollerGoal.GENTLE_INTAKE));
    }
}
