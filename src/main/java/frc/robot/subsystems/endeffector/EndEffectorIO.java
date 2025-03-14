package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import frc.robot.util.AutoClosing;
import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO extends AutoClosing {
  @AutoLog
  public static class EndEffectorInputs {
    // Motor Values
    public boolean positionMotorIsConnected = false;
    public MutAngle positionMotorPositionRad = Radians.mutable(0.0);
    public MutAngularVelocity positionMotorVelocityRadPerSec = RadiansPerSecond.mutable(0.0);
    public MutVoltage positionMotorAppliedVolts = Volts.mutable(0.0);
    public MutCurrent positionMotorCurrentAmps = Amps.mutable(0.0);

    // Effection Motor Values
    public boolean effectionMotorIsConnected = false;
    public MutAngle effectionMotorPositionRad = Radians.mutable(0.0);
    public MutAngularVelocity effectionMotorVelocityRadPerSec = RadiansPerSecond.mutable(0.0);
    public MutVoltage effectionMotorAppliedVolts = Volts.mutable(0.0);
    public MutCurrent effectionMotorCurrentAmps = Amps.mutable(0.0);

    public MutAngle endEffectorAbsolutePositionRad = Radians.mutable(0.0);
    public MutAngularVelocity enfEffectorAbsoluteVelocityRadPerSec = RadiansPerSecond.mutable(0.0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(EndEffectorInputs inputs) {}

  public default void setPositionMotorVoltage(Voltage voltage) {}

  public default void setEffectionMotorVoltage(Voltage voltage) {}
}
