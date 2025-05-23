package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;

public class ExitableCommand extends Command {
    private Command command;
    private BooleanSupplier condition;

    public ExitableCommand(Command command, BooleanSupplier condition) {
        this.command = command;
        this.condition = condition;

        addRequirements(command.getRequirements());
    }

    @Override
    public void initialize() {
        command.initialize();
    }

    @Override
    public void execute() {
        command.execute();
    }

    @Override
    public boolean isFinished() {
        return condition.getAsBoolean() || command.isFinished();
    }

    @Override
    public void end(boolean finished) {
        command.end(finished);
    }
}
