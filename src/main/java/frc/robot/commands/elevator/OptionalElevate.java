package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import java.util.function.Supplier;

public class OptionalElevate extends SequentialCommandGroup {

    public OptionalElevate(RobotContainer robot, Supplier<ElevatorGoal> goal) {
        var elevator = robot.getElevator();

        addCommands(new ConditionalCommand(
                new SafeElevate(robot, goal), new InstantCommand(), () -> elevator.getCurrentGoal() != goal.get()));
    }
}
