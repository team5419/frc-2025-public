package frc.robot.subsystems.pivot;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class PivotIOInputsAutoLogged extends PivotIO.PivotIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("MotorConnected", motorConnected);
    table.put("Position", position);
    table.put("AppliedVolts", appliedVolts);
    table.put("Velocity", velocity);
    table.put("TempCelcius", tempCelcius);
    table.put("SupplyCurrentAmps", supplyCurrentAmps);
    table.put("ReferencePose", referencePose);
    table.put("ReferenceVelocity", referenceVelocity);
  }

  @Override
  public void fromLog(LogTable table) {
    motorConnected = table.get("MotorConnected", motorConnected);
    position = table.get("Position", position);
    appliedVolts = table.get("AppliedVolts", appliedVolts);
    velocity = table.get("Velocity", velocity);
    tempCelcius = table.get("TempCelcius", tempCelcius);
    supplyCurrentAmps = table.get("SupplyCurrentAmps", supplyCurrentAmps);
    referencePose = table.get("ReferencePose", referencePose);
    referenceVelocity = table.get("ReferenceVelocity", referenceVelocity);
  }

  public PivotIOInputsAutoLogged clone() {
    PivotIOInputsAutoLogged copy = new PivotIOInputsAutoLogged();
    copy.motorConnected = this.motorConnected;
    copy.position = this.position;
    copy.appliedVolts = this.appliedVolts;
    copy.velocity = this.velocity;
    copy.tempCelcius = this.tempCelcius;
    copy.supplyCurrentAmps = this.supplyCurrentAmps;
    copy.referencePose = this.referencePose;
    copy.referenceVelocity = this.referenceVelocity;
    return copy;
  }
}
