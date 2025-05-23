package frc.robot.subsystems.roller;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class RollerIOInputsAutoLogged extends RollerIO.RollerIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("MotorConnected", motorConnected);
    table.put("MotorPositionRads", motorPositionRads);
    table.put("MotorVelocityRPS", motorVelocityRPS);
    table.put("MotorAppliedVolts", motorAppliedVolts);
    table.put("MotorSupplyCurrentAmps", motorSupplyCurrentAmps);
    table.put("MotorTorqueCurrentAmps", motorTorqueCurrentAmps);
    table.put("MotorTempCelsius", motorTempCelsius);
  }

  @Override
  public void fromLog(LogTable table) {
    motorConnected = table.get("MotorConnected", motorConnected);
    motorPositionRads = table.get("MotorPositionRads", motorPositionRads);
    motorVelocityRPS = table.get("MotorVelocityRPS", motorVelocityRPS);
    motorAppliedVolts = table.get("MotorAppliedVolts", motorAppliedVolts);
    motorSupplyCurrentAmps = table.get("MotorSupplyCurrentAmps", motorSupplyCurrentAmps);
    motorTorqueCurrentAmps = table.get("MotorTorqueCurrentAmps", motorTorqueCurrentAmps);
    motorTempCelsius = table.get("MotorTempCelsius", motorTempCelsius);
  }

  public RollerIOInputsAutoLogged clone() {
    RollerIOInputsAutoLogged copy = new RollerIOInputsAutoLogged();
    copy.motorConnected = this.motorConnected;
    copy.motorPositionRads = this.motorPositionRads;
    copy.motorVelocityRPS = this.motorVelocityRPS;
    copy.motorAppliedVolts = this.motorAppliedVolts;
    copy.motorSupplyCurrentAmps = this.motorSupplyCurrentAmps;
    copy.motorTorqueCurrentAmps = this.motorTorqueCurrentAmps;
    copy.motorTempCelsius = this.motorTempCelsius;
    return copy;
  }
}
