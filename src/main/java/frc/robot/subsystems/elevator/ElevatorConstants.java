// Copyright 2025 FRC 7540
// http://https://github.com/FRC7540
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Per;
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
  public static final boolean invertMotorB = true;

  public static final Distance minHieght = Meters.of(0);
  public static final Distance maxHieght = Meters.of(5);

  public static final double gerboxReduction = 1.0;

  /* Motion Definitions */
  public static final LinearVelocity homingSpeed = MetersPerSecond.of(0.25);

  /* Motion Unit Definitons */
  public static final Distance extensionFactor = Meters.of(1.0);
  public static final LinearVelocity velocityFactor = MetersPerSecond.of(1.0);
  public static final Per<DistanceUnit, AngleUnit> extensionConversionFactor =
      extensionFactor.div(Radians.one());
  public static final Per<LinearVelocityUnit, AngularVelocityUnit> velocityConversionFactor =
      velocityFactor.div(RadiansPerSecond.one());

  /* Other Definitons */
  public static final Time limitSwitchDebounceTime = Milliseconds.of(20);

  /* Control System Definitions */
  public static final Time nominalLoopTime = Milliseconds.of(20);
  public static final Matrix<N2, N1> stateCovarianceMatrix = VecBuilder.fill(3, 3);
  public static final Matrix<N2, N1> measurmentCovarianceMatrix = VecBuilder.fill(0.01, 0.01);

  public static final Distance maximumPositionExcusrsion = Meters.of(0.1);
  public static final LinearVelocity maximumVecloityExcursion = MetersPerSecond.of(0.1);
  public static final Vector<N2> stateExcursionToleranceMatrix =
      VecBuilder.fill(
          maximumPositionExcusrsion.in(Meters), maximumVecloityExcursion.in(MetersPerSecond));
  public static final Voltage controlAuthority = Volts.of(12.0);
  public static final Vector<N1> controlAuthorityMatrix =
      VecBuilder.fill(controlAuthority.in(Volts));
  /* Sim Definitons */
  public static final Angle simMaxAngleBackup = Radians.of(250);
  public static final String simMaxAngleKey = "Elevator/simMaxAngle";
  public static final Mass carraigeMass = Kilograms.of(0.5);
  public static final Distance drumRadius = Inches.of(1.893);

  public static final double simPositionStdDev = 0.05;
  public static final double simVelocityStdDev = 0.05;
}
