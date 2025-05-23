package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class RotateToSource extends Command {
    private static final double THETATOL = Math.PI / 90;

    private static final double THETA_MAX_VELOCITY = 6.0;
    private static final double THETA_MAX_ACCELERATION = 15.0;
    private static final double CONTROLLER_EXIT = 0.7;

    private static final double THETA_KP = 4;

    private RobotContainer robot;
    private Swerve swerve;
    private CommandXboxController xboxController;
    private BooleanSupplier slowMode;

    private ProfiledPIDController thetaController;

    public RotateToSource(RobotContainer robot, CommandXboxController controller, BooleanSupplier slowMode) {
        this.robot = robot;
        this.swerve = robot.getSwerve();
        this.xboxController = controller;
        this.slowMode = slowMode;
        addRequirements(robot.getSwerve());
        thetaController = new ProfiledPIDController(
                THETA_KP, 0.0, 0.0, new TrapezoidProfile.Constraints(THETA_MAX_VELOCITY, THETA_MAX_ACCELERATION));

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(THETATOL);
    }

    @Override
    public void initialize() {
        thetaController.reset(
                swerve.getPose().getRotation().getRadians(), swerve.getChassisSpeeds().omegaRadiansPerSecond);
    }

    @Override
    public void execute() {
        DoubleSupplier xSupplier = () -> robot.getDriver().getLeftY();
        DoubleSupplier ySupplier = () -> robot.getDriver().getLeftX();
        Supplier<Rotation2d> rotationSupplier = () -> swerve.getBestSourceAutoAlign();
        double slowModeMultiplier = slowMode.getAsBoolean() ? 0.25 : 1.0;

        Translation2d linearVelocity =
                DriveCommands.getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble(), false);

        double omega = thetaController.calculate(
                swerve.getRotation().getRadians(), rotationSupplier.get().getRadians());

        ChassisSpeeds speeds = new ChassisSpeeds(
                linearVelocity.getX() * swerve.getMaxLinearSpeedMetersPerSec() * slowModeMultiplier,
                linearVelocity.getY() * swerve.getMaxLinearSpeedMetersPerSec() * slowModeMultiplier,
                omega);
        boolean isFlipped = DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Blue;
        swerve.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds, isFlipped ? swerve.getRotation().plus(new Rotation2d(Math.PI)) : swerve.getRotation()));

        Logger.recordOutput("Rotate To Source/At Goal", thetaController.atGoal());
    }

    @Override
    public boolean isFinished() {
        return Math.abs(xboxController.getRightX()) > CONTROLLER_EXIT;
    }

    @Override
    public void end(boolean interrupted) {}
}
