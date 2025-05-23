package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.lib.LoggedTunableNumber;
import frc.robot.subsystems.swerve.Swerve;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;

public class DriveToPos extends Command {
    private static final double LINEARTOL = 0.15;
    private static final double THETATOL = 0.15;
    private static final double LINEARDEADBAND = 0.03;
    private static final double THETADEADBAND = 0.03;
    private static final double TOLTIMEOUT = .2;
    private static final double ANGLE_MAX_VELOCITY = Math.PI * 4;
    private static final double ANGLE_MAX_ACCELERATION = Math.PI * 4;
    private static final double LINEAR_MAX_ACCELERATION = 7;
    private static final double CONTROLLER_EXIT = 0.8;

    private static final double MAX_DISTANCE_TO_CORAL = 4.0;
    private static double MAX_SPEED_SCALAR = 0.8;
    private static double LINEAR_MAX_VELOCITY;
    // private static final double DISTANCE_WAIT_FOR_ELE = 0.5;

    private static final LoggedTunableNumber linearkP = new LoggedTunableNumber("AutoAlign/drivekP", 0.8);
    private static final LoggedTunableNumber linearkD = new LoggedTunableNumber("AutoAlign/drivekD", 0.05);
    private static final LoggedTunableNumber thetakP = new LoggedTunableNumber("AutoAlign/thetakP", 0.2);
    private static final LoggedTunableNumber thetakD = new LoggedTunableNumber("AutoAlign/thetakD", 0.0);

    private static final LoggedTunableNumber linearTolorance =
            new LoggedTunableNumber("AutoAlign/Linear Tolorance", LINEARTOL);
    private static final LoggedTunableNumber thetaTolorance =
            new LoggedTunableNumber("AutoAlign/Theta Tolorance", THETATOL);

    private final BooleanSupplier slowMode;
    private final BooleanSupplier isWarmup;
    private final ProfiledPIDController linearXController;
    private final ProfiledPIDController linearYController;
    private final ProfiledPIDController thetaController;
    private final Timer tolTimer;
    private Supplier<Pose2d> targetPose;
    private Swerve swerve;
    private CommandXboxController driver;

    public DriveToPos(
            RobotContainer robot,
            CommandXboxController driver,
            Supplier<Pose2d> pose,
            BooleanSupplier slowMode,
            BooleanSupplier isWarmup) {
        this.slowMode = slowMode;
        this.isWarmup = isWarmup;
        this.driver = driver;
        targetPose = pose;
        swerve = robot.getSwerve();
        addRequirements(swerve);
        LINEAR_MAX_VELOCITY = swerve.getMaxLinearSpeedMetersPerSec() * MAX_SPEED_SCALAR;

        linearXController = new ProfiledPIDController(
                linearkP.get(),
                0,
                linearkD.get(),
                new TrapezoidProfile.Constraints(swerve.getMaxLinearSpeedMetersPerSec(), LINEAR_MAX_ACCELERATION));

        linearYController = new ProfiledPIDController(
                linearkP.get(),
                0,
                linearkD.get(),
                new TrapezoidProfile.Constraints(LINEAR_MAX_VELOCITY, LINEAR_MAX_ACCELERATION));

        thetaController = new ProfiledPIDController(
                thetakP.get(),
                0,
                thetakD.get(),
                new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        tolTimer = new Timer();

        setTolorances(linearTolorance.get(), thetaTolorance.get());

        updateConstraints();
        resetControllers();
    }

    private void setTolorances(double linear, double theta) {
        linearXController.setTolerance(linear);
        linearYController.setTolerance(linear);
        thetaController.setTolerance(theta);
    }

    private void updateConstraints() {
        double scalar = slowMode.getAsBoolean() ? 0.25 : 1; // ! Slow mode?
        double maxSpeed = LINEAR_MAX_VELOCITY * scalar;
        linearXController.setConstraints(new TrapezoidProfile.Constraints(maxSpeed, LINEAR_MAX_ACCELERATION * scalar));
        linearYController.setConstraints(new TrapezoidProfile.Constraints(maxSpeed, LINEAR_MAX_ACCELERATION * scalar));
        thetaController.setConstraints(new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    }

    private void resetControllers() {
        Pose2d currentPose = swerve.getPose();
        ChassisSpeeds fieldVelocity = swerve.getChassisSpeeds();

        linearXController.reset(currentPose.getX(), fieldVelocity.vxMetersPerSecond);
        linearYController.reset(currentPose.getY(), fieldVelocity.vyMetersPerSecond);
        thetaController.reset(currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
    }

    @Override
    public void initialize() {

        RobotState.getInstance().setAutoAlignAtGoal(false);

        if (PhotonUtils.getDistanceToPose(swerve.getPose(), targetPose.get()) > MAX_DISTANCE_TO_CORAL) {
            this.cancel();
            return;
        }

        swerve.setIsAutoAligning(true);
        resetControllers();
        RobotState.getInstance().setAligning(true);
    }

    @SuppressWarnings("unused")
    private void updatePIDS(double linearP, double linearD, double thetaP, double thetaD) {
        linearXController.setPID(linearP, 0, linearD);
        linearYController.setPID(linearP, 0, linearD);
        thetaController.setPID(thetaP, 0, thetaD);
    }

    @Override
    public void execute() {

        Pose2d currentPose = swerve.getPose();

        double driveVelocityX = MathUtil.applyDeadband(
                linearXController.calculate(currentPose.getX(), targetPose.get().getX())
                        * swerve.getMaxLinearSpeedMetersPerSec(),
                LINEARDEADBAND);
        double driveVelocityY = MathUtil.applyDeadband(
                linearYController.calculate(currentPose.getY(), targetPose.get().getY())
                        * swerve.getMaxLinearSpeedMetersPerSec(),
                LINEARDEADBAND);
        double thetaVelocity = MathUtil.applyDeadband(
                thetaController.calculate(
                                currentPose.getRotation().getRadians(),
                                targetPose.get().getRotation().getRadians())
                        * swerve.getMaxAngularSpeedRadPerSec(),
                THETADEADBAND);
        ChassisSpeeds adjustedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                driveVelocityX, driveVelocityY, thetaVelocity, swerve.getPose().getRotation());

        swerve.runVelocity(adjustedSpeeds);

        if (!linearXController.atGoal() || !linearYController.atGoal() || !thetaController.atGoal()) {
            tolTimer.restart();
        }

        RobotState.getInstance().setAutoAlignAtGoal(tolTimer.hasElapsed(TOLTIMEOUT));

        if (isWarmup.getAsBoolean()) this.cancel();

        Logger.recordOutput("DriveToPose/DriveToPose Active", true);
        Logger.recordOutput("DriveToPose/LinearX At Goal", linearXController.atGoal());
        Logger.recordOutput("DriveToPose/LinearY At Goal", linearYController.atGoal());
        Logger.recordOutput("DriveToPose/Theta At Goal", thetaController.atGoal());
        Logger.recordOutput("DriveToPose/DriveVelocityX", driveVelocityX);
        Logger.recordOutput("DriveToPose/DriveVelocityY", driveVelocityY);
        Logger.recordOutput("DriveToPose/ThetaVelocity", thetaVelocity);
        Logger.recordOutput("DriveToPose/TargetPose", targetPose.get());
        Logger.recordOutput("DriveToPose/Tol Timer", tolTimer.get());
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(driver.getLeftX()) > CONTROLLER_EXIT
                        || Math.abs(driver.getLeftY()) > CONTROLLER_EXIT
                        || Math.abs(driver.getRightX()) > CONTROLLER_EXIT)
                || tolTimer.hasElapsed(TOLTIMEOUT);
    }

    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("DriveToPose/DriveToPose Active", false);
        RobotState.getInstance().setAligning(false);
        swerve.setIsAutoAligning(false);
    }
}
