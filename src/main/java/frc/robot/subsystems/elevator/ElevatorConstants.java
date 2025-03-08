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

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
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
  public static final Current elevatorMotorMaxCurrent = Amps.of(80);
  public static final Voltage elevatorMotorNominalVoltage = Volts.of(12.0);
  public static final boolean invertMotorB = false;

  public static final Distance minHieght = Meters.of(0);
  public static final Distance maxHieght = Meters.of(2.616);

  public static final Angle maxExtenesionRadians = Radians.of(150.0);
  public static final Angle minExtenesionRadians = Radians.of(0.0);

  public static final double gerboxReduction = 1.0;

  /* Motion Definitions */
  public static final LinearVelocity homingSpeed = MetersPerSecond.of(0.25);

  /* Motion Unit Definitons */
  public static final Distance extensionFactor = Meters.of(2.616);
  // 2.616 ( max extension) / 160 ( radian count at max extension)
  public static final LinearVelocity velocityFactor = MetersPerSecond.of(0.01635 / 100);
  public static final Per<DistanceUnit, AngleUnit> extensionConversionFactor =
      extensionFactor.div(Radians.of(160.0));
  public static final Per<LinearVelocityUnit, AngularVelocityUnit> velocityConversionFactor =
      velocityFactor.div(RadiansPerSecond.one());

  public static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(0.5);
  public static final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(1.0);

  /* Other Definitons */
  public static final Time limitSwitchDebounceTime = Milliseconds.of(20);

  /* Control System Definitions */
  public static final Time nominalLoopTime = Milliseconds.of(20);

  public class ControlLoopConstants {
    public static final double P = 8.143;
    public static final double I = 0.0;
    public static final double D = 0.454;
    public static final double S = 0.75151;
    public static final double G = 0.50194;
    public static final double V = 1.8601;
    public static final double A = 0.24819;
  }

  /* Sim Definitons */
  public static final Angle simMaxAngleBackup = Radians.of(250);
  public static final String simMaxAngleKey = "Elevator/simMaxAngle";
  public static final Mass carraigeMass = Kilograms.of(0.5);
  public static final Distance drumRadius = Inches.of(1.893);

  public static final double simPositionStdDev = 0.00005;
  public static final double simVelocityStdDev = 0.00005;
}
