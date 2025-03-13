package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public class EndEffectorConstants {

  /* Hardware Definitons */
  public static final Current effectionMotorMaxCurrent = Amps.of(20);
  public static final Voltage effectionMotorNominalVoltage = Volts.of(12.0);

  public static final Current positonalMotorMaxCurrent = Amps.of(50);
  public static final Voltage positonalMotorNominalVoltage = Volts.of(12.0);

  /* Motion Definitions */
  public static final double encoderPositionFactor = 1.0;
  public static final double encoderVelocityFactor = 1.0;

  public static final double positonalDriveGearing = (26.0 / 14.0) * 12.0;
  public static final double positonalDriveMotorEndcoderVelocityFactor = 1 / positonalDriveGearing;
  public static final double positonalDriveMotorEndcoderPositionFactor = 1 / positonalDriveGearing;
  public static final Angle positonEncoderOffset =
      Radians.of((2.75 - 2.50) + Math.PI); // Math.PI * 1 - 2.6794

  public static final MomentOfInertia mechanismMOI = KilogramSquareMeters.of(0.075);
  public static final Angle minAngle = Radians.of(2.2);
  public static final Angle maxAngle = Radians.of(4.7);

  /* Control System Definitions */
  public static final Time nominalLoopTime = Milliseconds.of(20);
  public static final Matrix<N2, N1> stateCovarianceMatrix = VecBuilder.fill(3.0, 3.0);
  public static final Matrix<N2, N1> measurmentCovarianceMatrix = VecBuilder.fill(0.5, 0.5);

  public static final Angle maximumPositionExcusrsion = Radians.of(0.01);
  public static final AngularVelocity maximumVecloityExcursion = RadiansPerSecond.of(0.001);

  public static final Voltage minControlAuthority = Volts.of(8.0);
  public static final Voltage maxControlAuthority = Volts.of(-4.0);

  /* Derived Constants */
}
