package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoScore;

public class DisplaceAndScore extends SequentialCommandGroup {
    public DisplaceAndScore(RobotContainer robot, CommandXboxController controller) {

        var elevator = robot.getElevator();
        var pivot = robot.getPivot();
        var swerve = robot.getSwerve();
        addRequirements(elevator, pivot, swerve);

        addCommands(new DisplaceAlgae(robot, controller), new AutoScore(robot, controller));
    }
}
