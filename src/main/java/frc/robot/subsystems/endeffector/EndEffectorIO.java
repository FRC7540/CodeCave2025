package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.AutoClosing;
import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO extends AutoClosing {
  @AutoLog
  public static class EndEffectorInputs {
    // Endstops
    public boolean lowerLimitSwitch = false;
    public boolean upperLimitSwitch = false;

    // Motor Values
    public boolean motorIsConnected = false;
    public double motorPositionRad = 0.0;
    public double motorVelocityRadPerSec = 0.0;
    public MutVoltage motorAppliedVolts = Volts.mutable(0.0);
    public MutCurrent motorCurrentAmps = Amps.mutable(0.0);

    public double endEffectorAbsolutePositionRad = 0.0;
    public double enfEffectorAbsoluteVelocityRadPerSec = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(EndEffectorInputs inputs) {}

  public default void setMotorVoltage(Voltage voltage) {}
}
