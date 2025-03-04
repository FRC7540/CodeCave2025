package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
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
  public static final int effectionMotorCANID = 12;
  public static final Current effectionMotorMaxCurrent = Amps.of(20);
  public static final Voltage effectionMotorNominalVoltage = Volts.of(12.0);

  public static final int positonalMotorCANID = 13;
  public static final Current positonalMotorMaxCurrent = Amps.of(40);
  public static final Voltage positonalMotorNominalVoltage = Volts.of(12.0);

  /* Motion Definitions */
  public static final double encoderPositionFactor = 1.0;
  public static final double encoderVelocityFactor = 1.0;
  public static final double positonalDriveGearing = (26.0 / 14.0) * 12.0;
  public static final double positonalDriveMotorEndcoderVelocityFactor = 1 / positonalDriveGearing;
  public static final double positonalDriveMotorEndcoderPositionFactor = 1 / positonalDriveGearing;
  public static final Angle positonEncoderOffset = Radians.of(2.75);

  public static final MomentOfInertia mechanismMOI = KilogramSquareMeters.of(0.071);
  public static final Angle minAngle = Radians.of(2.1);
  public static final Angle maxAngle = Radians.of(4.4);

  /* Control System Definitions */
  public static final Time nominalLoopTime = Milliseconds.of(20);
  public static final Matrix<N2, N1> stateCovarianceMatrix = VecBuilder.fill(3.0, 3.0);
  public static final Matrix<N2, N1> measurmentCovarianceMatrix = VecBuilder.fill(0.5, 0.5);

  public static final Angle maximumPositionExcusrsion = Radians.of(0.01);
  public static final AngularVelocity maximumVecloityExcursion = RadiansPerSecond.of(0.01);

  public static final Voltage controlAuthority = Volts.of(12.0);

  /* Derived Constants */
  public static final Vector<N1> controlAuthorityMatrix =
      VecBuilder.fill(controlAuthority.in(Volts));
  public static final Vector<N2> stateExcursionToleranceMatrix =
      VecBuilder.fill(
          maximumPositionExcusrsion.in(Radians), maximumVecloityExcursion.in(RadiansPerSecond));
}
