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

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.AutoClosing;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase implements AutoClosing {
  private final ElevatorIO elevatorIO;
  private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
  private final SysIdRoutine sysIdRoutine;

  private final Alert notHomedAlert =
      new Alert(
          "Elevator Must be Homed before running any control loop operations!", AlertType.kError);

  /* Elevator State */
  @AutoLogOutput(key = "Elevator/extension")
  private MutDistance elevatorExtension = Meters.mutable(0.0);

  @AutoLogOutput(key = "Elevator/velocity")
  private MutLinearVelocity elevatorVelocity = MetersPerSecond.mutable(0.0);

  /* displacment: Desired Elevator Extension */
  @AutoLogOutput(key = "Elevator/reference")
  private MutDistance positionReference = Meters.mutable(0.0);

  /* whether or not we have been homed */
  @AutoLogOutput(key = "Elevator/isHomed")
  private boolean isHomed = false;

  /* Control Systems */

  private final ElevatorFeedforward feedforward =
      new ElevatorFeedforward(
          ElevatorConstants.ControlLoopConstants.S,
          ElevatorConstants.ControlLoopConstants.G,
          ElevatorConstants.ControlLoopConstants.V,
          ElevatorConstants.ControlLoopConstants.A);
  private final ProfiledPIDController feedback =
      new ProfiledPIDController(
          ElevatorConstants.ControlLoopConstants.P,
          ElevatorConstants.ControlLoopConstants.I,
          ElevatorConstants.ControlLoopConstants.D,
          new TrapezoidProfile.Constraints(
              ElevatorConstants.MAX_VELOCITY.in(MetersPerSecond),
              ElevatorConstants.MAX_ACCELERATION.in(MetersPerSecondPerSecond)));

  /* Should we be runnning the control system? */
  @AutoLogOutput(key = "Elevator/controlSystemActive")
  private boolean controlSystemActive = false;

  public Elevator(ElevatorIO elevatorIO) {
    this.elevatorIO = elevatorIO;
    SmartDashboard.putData("PIDSS", feedback);
    // Create the SysId routine
    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(1.75),
                null, // Use default config
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> this.runVolts(voltage),
                null, // No log consumer, since data is recorded by AdvantageKit
                this));

    /* Initalize Values */
    this.elevatorExtension.mut_replace(Meters.of(0.0));
    this.elevatorVelocity.mut_replace(MetersPerSecond.of(0.0));
    this.positionReference.mut_replace(Meters.of(0.0));
  }

  @Override
  public void periodic() {
    elevatorIO.updateInputs(elevatorInputs);
    Logger.processInputs("Elevator", elevatorInputs);

    /* Clear Alerts that can't possibly be valid */
    if (this.isHomed) {
      this.notHomedAlert.set(false);
    }

    // Calculate derived varibles
    elevatorExtension.mut_replace(
        elevatorInputs.A.positionRad.timesConversionFactor(
            ElevatorConstants.extensionConversionFactor));
    elevatorVelocity.mut_replace(
        elevatorInputs.A.velocityRadPerSec.timesConversionFactor(
            ElevatorConstants.velocityConversionFactor));

    // Determine desired position / state

    if (!this.controlSystemActive) {
      return;
    }

    double cvolt = feedback.calculate(elevatorExtension.in(Meters), positionReference.in(Meters));
    cvolt += feedforward.calculate(feedback.getSetpoint().velocity);

    elevatorIO.setMotorVoltage(Volts.of(cvolt));
  }

  /**
   * Enables or disabled active control of the elevator by the elevator subsystem
   *
   * @param active If true, the control system will be activated and we will drive the elevator to
   *     the current setpoints
   */
  public void setControlsActive(boolean active) {
    if (!isHomed) {
      this.notHomedAlert.set(true);
    }
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
   * Set wether or not we have been homed already
   *
   * @param active If true, the control system will be activated and we will drive the elevator to
   *     the current setpoints
   */
  public void setHomed(boolean active) {
    this.isHomed = active;
    this.notHomedAlert.set(false);
  }

  /**
   * Tells us wether or not we have been homed already
   *
   * @see setControlsActive to set the current state of the control system
   */
  public boolean getHomed() {
    return this.isHomed;
  }

  /**
   * Drives the elevator to a specified
   *
   * @param hieght Desired distance to a desired extension
   */
  public void setPosition(Distance hieght) {
    positionReference.mut_replace(hieght);
  }

  /**
   * Drives the elevator at a specicfic velocity
   *
   * @param speed Desired elevator velocity
   */
  public void setVelocity(LinearVelocity speed) {
    if (this.controlSystemActive == true) {
      DriverStation.reportWarning(
          "You must deactivate the elevator control systems before driving by voltage!", false);
      return;
    }
    elevatorIO.setMotorVoltage(Volts.of(feedforward.calculate(speed.in(MetersPerSecond))));
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
          "You must deactivate the elevator control systems before driving by voltage!", false);
      return;
    }
    elevatorIO.setMotorVoltage(voltage);
  }

  public void runVolts(Voltage voltage) {
    this.setControlsActive(false);
    this.driveVoltage(voltage);
  }

  /**
   * @return Status of the elevator lower limit switch
   */
  public boolean getLowerLimitSwitch() {
    return this.elevatorInputs.lowerLimitSwitch;
  }

  /**
   * @return Status of the elevator upper limit switch
   */
  public boolean getUpperLimitSwitch() {
    return this.elevatorInputs.upperLimitSwitch;
  }

  /**
   * @return Status of the elevator upper limit switch
   */
  public boolean getUpperSoftLimit() {
    return this.elevatorInputs.A.positionRad.gte(ElevatorConstants.maxExtenesionRadians);
  }

  /**
   * @return Status of the elevator upper limit switch
   */
  public boolean getLowerSoftLimit() {
    return this.elevatorInputs.A.positionRad.lte(ElevatorConstants.minExtenesionRadians);
  }

  public Distance getExtension() {
    return elevatorExtension;
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

  public void setZero() {
    this.elevatorIO.setZero();
  }
}
