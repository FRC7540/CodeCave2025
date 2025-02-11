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

package frc.robot.subsystems.endeffector;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.AutoClosing;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase implements AutoClosing {
  private final EndEffectorIO endeffectorio;
  private final EndEffectorInputsAutoLogged endeffectorinputs = new EndEffectorInputsAutoLogged();
  private final SysIdRoutine sysIdRoutine;

  /* Should we be runnning the control system? */
  @AutoLogOutput(key = "EndEffector/controlSystemActive")
  private boolean controlSystemActive;

  public EndEffector(EndEffectorIO endeffectorio) {
    this.endeffectorio = endeffectorio;

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null, // Use default config
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> this.runVolts(voltage),
                null, // No log consumer, since data is recorded by AdvantageKit
                this));
  }

  @Override
  public void periodic() {
    endeffectorio.updateInputs(endeffectorinputs);
    Logger.processInputs("EndEffector", endeffectorinputs);

    /* Determine What to feed the control loops */

    /* Feed the loops */

    /* Apply the values */
  }

  public void setTargetPosition(double targetPosition) {}

  /**
   * Enables or disabled active control of the elevator by the elevator subsystem
   *
   * @param active If true, the control system will be activated and we will drive the elevator to
   *     the current setpoints
   */
  public void setControlsActive(boolean active) {
    this.controlSystemActive = active;
  }

  /**
   * Gets the current status of the elevator control system
   *
   * @see setControlsActive to set the current state of the control system
   */
  public boolean getControlsActive() {
    return this.controlSystemActive;
  }

  /**
   * Drives the elevator motors at a specified voltage, used mainly for testing/tuning
   *
   * @see setControlsActive The elevator control system must be disabled for you to use this
   *     function!
   * @param voltage Elevator motor voltages
   */
  public void driveVoltage(Voltage voltage) {
    if (this.controlSystemActive == true) {
      DriverStation.reportWarning(
          "You must deactivate the end effector control systems before driving by voltage!", false);
      return;
    }
    endeffectorio.setMotorVoltage(voltage);
  }

  /**
   * Disables the control system gaurd and runs at the specified voltage
   *
   * @param voltage voltage to drive motor at
   */
  public void runVolts(Voltage voltage) {
    this.setControlsActive(false);
    endeffectorio.setMotorVoltage(voltage);
  }

  /**
   * @param direction SysIdDirection for routine
   * @return Command that runs the sysid routine requested
   */
  public Command sysIDQuasistatic(SysIdRoutine.Direction direction) {
    // The methods below return Command objects
    return sysIdRoutine.quasistatic(direction);
  }

  /**
   * @param direction SysIdDirection for routine
   * @return Command that runs the sysid routine requested
   */
  public Command sysIDDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
}
