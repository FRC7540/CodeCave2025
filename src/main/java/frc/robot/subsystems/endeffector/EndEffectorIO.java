package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
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
    public MutAngle motorPositionRad = Radians.mutable(0.0);
    public MutAngularVelocity motorVelocityRadPerSec = RadiansPerSecond.mutable(0.0);
    public MutVoltage motorAppliedVolts = Volts.mutable(0.0);
    public MutCurrent motorCurrentAmps = Amps.mutable(0.0);

    public MutAngle endEffectorAbsolutePositionRad = Radians.mutable(0.0);
    public MutAngularVelocity enfEffectorAbsoluteVelocityRadPerSec = RadiansPerSecond.mutable(0.0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(EndEffectorInputs inputs) {}

  public default void setMotorVoltage(Voltage voltage) {}
}
