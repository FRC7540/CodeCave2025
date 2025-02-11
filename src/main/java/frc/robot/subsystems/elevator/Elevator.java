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

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO elevatorIO;
  private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();

  /* Elevator State */
  /* extensionPercentage: a Value between 0 and 1 that represents the current extension of the elevator */
  @AutoLogOutput(key = "Elevator/extensionPercentage")
  private double extensionPercentage;

  /* displacment: The current displacment of the carraige, from the home position (Fully Retracted) */
  @AutoLogOutput(key = "Elevator/displacment")
  private Distance displacment;

  /* groundExtension: the current distance from the ground to the middle of the elevator carraige */
  @AutoLogOutput(key = "Elevator/groundExtension")
  private Distance groundExtension;

  /* Control Systems */

  /* Should we be runnning the control system? */
  @AutoLogOutput(key = "Elevator/controlSystemActive")
  private boolean controlSystemActive;

  /* This plant holds a model of our elevator, the system has the following properties:
   *
   * States: [Position, Velocity] (Of the elevator)
   * Inputs: [Voltage] (Input to the plant)
   * Outputs: [Position] (Current Position of the plant)
   */

  public Elevator(ElevatorIO elevatorIO) {
    this.elevatorIO = elevatorIO;
  }

  @Override
  public void periodic() {
    elevatorIO.updateInputs(elevatorInputs);
    Logger.processInputs("Elevator", elevatorInputs);
    // Calculate derived varibles

    // For now...
    double positionRadians = 0.0;
    extensionPercentage = calculateExtensionPercentage(positionRadians);
    displacment = calculateDisplacement(extensionPercentage);
    groundExtension = calculateGroundExtension(extensionPercentage);

    // Determine desired position / state

    // Run Control Loops

    // Apply Control Loops
  }

  /**
   * Calculate elevator extension percentage
   *
   * @param encoderRadianMeasurment The current measrument from the elevator encoder readings
   * @return [0.0 <-> 1.0] The current extension percentage of the elevator
   */
  private double calculateExtensionPercentage(double encoderRadianMeasurment) {
    return 0.0;
  }

  /**
   * Calculate elevator extension percentage
   *
   * @param groundExtension The current measrument of elevator ground extension
   * @return [0.0 <-> 1.0] The current extension percentage of the elevator
   */
  private double calculateExtensionPercentageFromGroundExtension(Distance groundExtension) {
    return 0.0;
  }

  /**
   * Calculate elevator extension percentage
   *
   * @param displacement The current measurment of elevator Displacement
   * @return [0.0 <-> 1.0] The current extension percentage of the elevator
   */
  private double calculateExtensionPercentageFromDisplacement(Distance displacement) {
    return 0.0;
  }

  /**
   * Calculate elevator displacment
   *
   * @param extensionPercentage a Value between 0 and 1 that represents the current extension of the
   *     elevator
   * @return Distance of elevator displacment, in WPILIB Units of "Distance"
   */
  private Distance calculateDisplacement(double extensionPercentage) {
    return Meters.of(0.0);
  }

  /**
   * Calculate elevator ground extension
   *
   * @param extensionPercentage a Value between 0 and 1 that represents the current extension of the
   *     elevator
   * @return Distance of elevator ground extension, in WPILIB Units of "Distance"
   */
  private Distance calculateGroundExtension(double extensionPercentage) {
    return Meters.of(0.0);
  }

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
   * Drives the elevator to a specified
   *
   * @param hieght Desired distance from ground to middle of elevator carraige
   */
  public void driveGroundExtension(Distance hieght) {
    System.out.println(
        "Method: driveGroundExtension - Not Implemented Yet! " + this.getSubsystem());
  }

  /**
   * Drives the elevator at a specicfic velocity
   *
   * @param speed Desired elevator velocity
   */
  public void driveSpeed(LinearVelocity speed) {
    System.out.println("Method: driveSpeed - Not Implemented Yet! " + this.getSubsystem());
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
}
