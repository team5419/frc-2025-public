// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {

    @AutoLog
    class RollerIOInputs {
        public boolean motorConnected = true;

        public double motorPositionRads = 0.0;
        public double motorVelocityRPS = 0.0;
        public double motorAppliedVolts = 0.0;
        public double motorSupplyCurrentAmps = 0.0;
        public double motorTorqueCurrentAmps = 0.0;
        public double motorTempCelsius = 0.0;
    }

    /** Update inputs */
    default void updateInputs(RollerIOInputs inputs) {}

    /** Run both motors at voltage */
    default void runVolts(double motorVolts) {}

    /** Stop both motors */
    default void stop() {}

    /** Run left and right motors at velocity in rpm */
    default void runVelocity(double motorRPS, double ff) {}

    /** Config PID values for both motors */
    default void setPID(double kP, double kI, double kD) {}
}
