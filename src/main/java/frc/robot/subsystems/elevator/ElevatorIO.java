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
    public double motorAAppliedVolts = 0.0;
    public double motorACurrentAmps = 0.0;

    public boolean motorBIsConnected = false;
    public double motorBPositionRad = 0.0;
    public double motorBVelocityRadPerSec = 0.0;
    public double motorBAppliedVolts = 0.0;
    public double motorBCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setMotorVoltage(double voltage) {}
}
