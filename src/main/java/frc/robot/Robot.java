// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.GlobalConstants;
import frc.robot.lib.VirtualSubsystem;
import frc.robot.subsystems.SimVisualizer;
import frc.robot.subsystems.leds.Leds;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

    private RobotContainer robotContainer;

    private SimVisualizer simVisualizer;

    private Command autonomousCommand;
    public PowerDistribution PDH;

    private Timer autoTimer = new Timer();
    private Timer disabledTimer = new Timer();
    private Timer canErrorTimer = new Timer();
    private Timer CANivoreErrorTimer = new Timer();
    private Timer CANInitialErrorTimer = new Timer();

    private final Alert canErrorAlert = new Alert("CAN errors detected", Alert.AlertType.kError);
    private final Alert lowBatteryAlert = new Alert("Battery is low. Please replace it", Alert.AlertType.kWarning);
    private final Alert CANivoreErrorAlert = new Alert("CANivore errors detected", Alert.AlertType.kError);
    private final Alert driverDisconnected = new Alert("Driver controller is disconnected", Alert.AlertType.kError);
    private final Alert codriverDisconnected = new Alert("Codriver controller is disconnected", Alert.AlertType.kError);

    private double autoTime;

    @Override
    public void robotInit() {

        RobotController.setBrownoutVoltage(GlobalConstants.kBrownoutVoltage);

        handleADKStartup();
        enableActiveCommandLogging();

        autoTimer.start();
        disabledTimer.start();
        CANivoreErrorTimer.start();
        CANInitialErrorTimer.start();
        canErrorTimer.start();

        robotContainer = new RobotContainer();
        simVisualizer = new SimVisualizer(robotContainer);

        Leds.getInstance();
        RobotState.getInstance();

        FollowPathCommand.warmupCommand().schedule();
    }

    @Override
    public void robotPeriodic() {

        if (Robot.isReal()) Threads.setCurrentThreadPriority(true, 99);

        driverDisconnected.set(!DriverStation.isJoystickConnected(0));
        codriverDisconnected.set(!DriverStation.isJoystickConnected(1));

        CommandScheduler.getInstance().run();
        VirtualSubsystem.periodicAll();
        Logger.recordOutput(
                "AutoAlign/Auto Align At Goal", RobotState.getInstance().isAutoAlignAtGoal());

        if (!DriverStation.isAutonomousEnabled()) {
            autoTimer.restart();
        }

        if (DriverStation.isEnabled()) {
            disabledTimer.restart();
        }

        CANStatus canStatus = RobotController.getCANStatus();
        if (canStatus.transmitErrorCount > 0 || canStatus.receiveErrorCount > 0) {
            canErrorTimer.restart();
        }

        canErrorAlert.set(!CANInitialErrorTimer.hasElapsed(GlobalConstants.kCANErrorTimeThreshold)
                && !canErrorTimer.hasElapsed(GlobalConstants.kCANErrorTimeThreshold));

        if (GlobalConstants.getMode() == GlobalConstants.Mode.REAL) {

            CANBus.CANBusStatus canivoreStatus = new CANBus(GlobalConstants.kCANivoreName).getStatus();

            Logger.recordOutput("CANivore/Status", canivoreStatus.Status);
            Logger.recordOutput("CANivore/Utilization", canivoreStatus.BusUtilization);
            Logger.recordOutput("CANivore/OffCount", canivoreStatus.BusOffCount);
            Logger.recordOutput("CANivore/TxFullCount", canivoreStatus.TxFullCount);
            Logger.recordOutput("CANivore/ReceiveErrorCount", canivoreStatus.REC);
            Logger.recordOutput("CANivore/TransmitErrorCount", canivoreStatus.TEC);

            if (!canivoreStatus.Status.isOK() || canivoreStatus.REC > 0 || canivoreStatus.TEC > 0) {
                CANInitialErrorTimer.restart();
            }

            CANivoreErrorAlert.set(!CANInitialErrorTimer.hasElapsed(GlobalConstants.kCANivoreErrorTimeThreshold)
                    && !CANivoreErrorTimer.hasElapsed(GlobalConstants.kCANivoreErrorTimeThreshold));
        }

        if (RobotController.getBatteryVoltage() < GlobalConstants.kLowBatteryVoltage
                && disabledTimer.hasElapsed(GlobalConstants.kLowBatteryDisabledTime)) {
            lowBatteryAlert.set(true);
        }

        Logger.recordOutput("Robot/AutoTime", autoTimer.get());
        Logger.recordOutput("Robot/DisabledTime", disabledTimer.get());
        Logger.recordOutput("Auto/Auto Time To Score", autoTime);
        Logger.recordOutput("Auto/Auto Is Finished", RobotState.getInstance().isAutoFinished());

        simVisualizer.update();

        Threads.setCurrentThreadPriority(true, 10);
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        RobotState.getInstance().setAutoFinished(false);
        autonomousCommand = robotContainer.getAutonomousCommand();
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
        if (!RobotState.getInstance().isAutoFinished()) autoTime = autoTimer.get();
    }

    @Override
    public void autonomousExit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopInit() {
        RobotState.getInstance().setAutoFinished(true);
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}

    private void handleADKStartup() {

        Logger.recordMetadata("Robot", GlobalConstants.getRobotType().toString());
        Logger.recordMetadata("TuningMode", Boolean.toString(GlobalConstants.kTuningMode));
        Logger.recordMetadata("DevelopmentMode", Boolean.toString(GlobalConstants.kDevMode));
        Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncommitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        // only one of NT or RLOG
        // Set up data receivers & replay source
        switch (GlobalConstants.getMode()) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                // Logger.addDataReceiver(new RLOGServer());

                break;

            case SIM:
                // Running a physics simulator, log to NT
                // Logger.addDataReceiver(new RLOGServer());
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")));
                break;
        }

        // Start AdvantageKit logger
        Logger.start();
    }

    private void enableActiveCommandLogging() {

        Map<String, Integer> commandCounts = new HashMap<>();
        BiConsumer<Command, Boolean> logCommandFunction = (Command command, Boolean active) -> {
            String name = command.getName();
            int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
            commandCounts.put(name, count);
            Logger.recordOutput("CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
            Logger.recordOutput("CommandsAll/" + name, count > 0);
        };

        CommandScheduler.getInstance().onCommandInitialize((Command command) -> {
            logCommandFunction.accept(command, true);
        });
        CommandScheduler.getInstance().onCommandFinish((Command command) -> {
            logCommandFunction.accept(command, false);
        });
        CommandScheduler.getInstance().onCommandInterrupt((Command command) -> {
            logCommandFunction.accept(command, false);
        });
    }
}
