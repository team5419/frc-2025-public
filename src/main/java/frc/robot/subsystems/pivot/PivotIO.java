package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    @AutoLog
    class PivotIOInputs {
        public boolean motorConnected = true;
        public double position = 0.0; // degrees
        public double appliedVolts = 0.0;
        public double velocity = 0.0; // degrees per second
        public double tempCelcius = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double referencePose = 0.0;
        public double referenceVelocity = 0.0;
    }

    default void updateInputs(PivotIOInputs inputs) {}

    default void runPosition(double goal) {} // sets the arm to a specific position

    default void runVolts(double volts) {}

    default void resetPosition(double angle) {}

    default void setBrakeMode(boolean enabled) {}

    default void setPID(double P, double I, double D) {}

    default void setFF(double kA, double kG, double kS, double kV) {}

    default void stop() {}
}
