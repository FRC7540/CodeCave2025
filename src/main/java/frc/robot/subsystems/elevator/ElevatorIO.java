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

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutTemperature;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.AutoClosing;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO extends AutoClosing {

  @AutoLog
  public static class ElevatorIOInputs {
    // Endstops
    public boolean lowerLimitSwitch = false;
    public boolean upperLimitSwitch = false;

    // Motor Values
    public boolean motorAIsConnected = false;
    public MutAngle motorAPositionRad = Radians.mutable(0);
    public MutAngularVelocity motorAVelocityRadPerSec = RadiansPerSecond.mutable(0);
    public MutVoltage motorAAppliedVolts = Volts.mutable(0.0);
    public MutCurrent motorACurrentAmps = Amps.mutable(0.0);
    public MutTemperature motorATemperature = Celsius.mutable(0.0);

    public boolean motorBIsConnected = false;
    public MutAngle motorBPositionRad = Radians.mutable(0);
    public MutAngularVelocity motorBVelocityRadPerSec = RadiansPerSecond.mutable(0);
    public MutVoltage motorBAppliedVolts = Volts.mutable(0.0);
    public MutCurrent motorBCurrentAmps = Amps.mutable(0.0);
    public MutTemperature motorBTemperature = Celsius.mutable(0.0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setMotorVoltage(Voltage voltage) {}

  public default void setZero() {}
}
