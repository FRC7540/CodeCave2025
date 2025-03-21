package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public static class ClimberIOInputs {
    public boolean motorIsConnected = false;
    public MutAngle motorPositionRad = Radians.mutable(0);
    public MutAngularVelocity motorVelocityRadPerSec = RadiansPerSecond.mutable(0);
    public MutVoltage motorAppliedVolts = Volts.mutable(0.0);
    public MutCurrent motorCurrentAmps = Amps.mutable(0.0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setMotorVoltage(Voltage voltage) {}

  public default void setZero() {}
}
