package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorConstants {
  /* Hardware Definitons */
  public static final int motorACANID = 10;
  public static final int motorBCANDID = 11;
  public static final int lowerLimitDIOID = 0;
  public static final int upperLimitDIOID = 1;
  public static final Current elevatorMotorMaxCurrent = Amps.of(40);
  public static final Voltage elevatorMotorNominalVoltage = Volts.of(12.0);

  /* Motion Definitions */
  public static final LinearVelocity homingSpeed = MetersPerSecond.of(0.25);
  public static final double encoderPositionFactor = 1.0;
  public static final double encoderVelocityFactor = 1.0;

  /* Other Definitons */
  public static final Time limitSwitchDebounceTime = Milliseconds.of(20);
}
