package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.Roller.RollerGoal;
import java.util.function.Supplier;

public class ElevateToPosition extends Command {
    private final Elevator elevator;
    private final Roller roller;
    private final Supplier<ElevatorGoal> positionSupplier;

    public ElevateToPosition(Elevator elevator, Roller roller, Supplier<ElevatorGoal> goal) {
        this.elevator = elevator;
        this.roller = roller;
        positionSupplier = goal;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        elevator.setCurrentGoal(positionSupplier.get());
        if (elevator.isGoalHigh()) roller.setCurrentGoal(RollerGoal.GENTLE_INTAKE);
    }

    @Override
    public void end(boolean interrupted) {
        roller.setCurrentGoal(RollerGoal.IDLE);
    }

    @Override
    public boolean isFinished() {
        return elevator.getCurrentGoal() == positionSupplier.get() && elevator.atGoal();
    }
}
