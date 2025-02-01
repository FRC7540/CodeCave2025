package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;

public class ElevatorConstants {
  /* Hardware Definitons */
  public static final int motorACANID = 10;
  public static final int motorBCANDID = 11;
  public static final int lowerLimitDIOID = 0;
  public static final int upperLimitDIOID = 1;

  /* Motion Definitions */
  public static final LinearVelocity homingSpeed = MetersPerSecond.of(0.25);
}
