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

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    // Endstops
    public boolean lowerLimitSwitch = false;
    public boolean upperLimitSwitch = false;

    // Motor Values
    public boolean motorAIsConnected = false;
    public double motorAPositionRad = 0.0;
    public double motorAVelocityRadPerSec = 0.0;
    public MutVoltage motorAAppliedVolts = Volts.mutable(0.0);
    public MutCurrent motorACurrentAmps = Amps.mutable(0.0);

    public boolean motorBIsConnected = false;
    public double motorBPositionRad = 0.0;
    public double motorBVelocityRadPerSec = 0.0;
    public MutVoltage motorBAppliedVolts = Volts.mutable(0.0);
    public MutCurrent motorBCurrentAmps = Amps.mutable(0.0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setMotorVoltage(Voltage voltage) {}
}
