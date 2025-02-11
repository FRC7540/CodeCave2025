package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

public class EndEffectorConstants {

  public static final int motorCANID = 13;
  public static final Current motorMaxCurrent = Amps.of(40);
  public static final Voltage motorNominalVoltage = Volts.of(12.0);

  public static final double encoderPositionFactor = 1.0;
  public static final double encoderVelocityFactor = 1.0;

  public static final LinearVelocity driveSpeed = MetersPerSecond.of(0.25);
}
