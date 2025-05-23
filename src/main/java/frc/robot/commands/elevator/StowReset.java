package frc.robot.commands.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.pivot.PivotReset;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.pivot.Pivot;

public class StowReset extends SequentialCommandGroup {

    private static Elevator elevator;
    private static Pivot pivot;

    public StowReset(RobotContainer robot) {

        elevator = robot.getElevator();
        pivot = robot.getPivot();
        addRequirements(elevator, pivot);

        addCommands(
                new SafeElevate(robot, () -> ElevatorGoal.STOW),
                new WaitUntilCommand(() -> elevator.getPosition() < kStowTolerance),
                new ParallelCommandGroup(
                        new PivotReset(robot),
                        new SequentialCommandGroup(
                                new InstantCommand(() -> elevator.runVolts(kStowVolts)),
                                new WaitCommand(0.02),
                                new WaitUntilCommand(() -> Math.abs(elevator.getAvgVelocity()) <= kIsStopped),
                                new InstantCommand(() -> elevator.stop()),
                                new WaitCommand(0.25),
                                new InstantCommand(() -> elevator.resetPosition()))));
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}
