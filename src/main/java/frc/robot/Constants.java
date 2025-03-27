// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
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

package frc.robot;

import static edu.wpi.first.units.Units.Celsius;

import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public final class HID {
    public final class RumbleStrengths {
      public static final double NUDGE_RUMBLE = 0.25;
      public static final double GENTLE_RUMBLE = 0.5;
      public static final double STRONG_RUBMLE = 0.75;
      public static final double VERY_STRONG_RUMBLE = 1.0;
    }

    public final class ControllerInterfaces {
      public static final int DRIVER_CONTROLLER_ID = 0;
      public static final int OPERATOR_CONTROLLER_ID = 1;
    }

    public final class SensitivityMultipliers {
      public static final double DriveSpeedSlowModeMultiplier = 0.5;
      public static final double EndEffectorJoystickControlSensitivity = 0.1;
      public static final double ElevatorJoystickControlSensitivity = 0.1;
    }
  }

  public static final Temperature warnNeoOneTemp = Celsius.of(120);
  public static final Temperature warnNeoFiveFiftyTemp = Celsius.of(130);

  public static final Temperature criticalNeoOneTemp = Celsius.of(120);
  public static final Temperature criticalNeoFiveFiftyTemp = Celsius.of(130);

  public static final Temperature warnVortexTemp = Celsius.of(120);
  public static final Temperature criticalVortexTemp = Celsius.of(130);

  public static final Mode simMode = Mode.SIM;

  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
}
