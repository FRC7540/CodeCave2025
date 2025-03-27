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

package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.AutoClosing;
import org.littletonrobotics.junction.Logger;

public class Module implements AutoClosing {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  private Trigger driveMotorOverheat =
      new Trigger(
          Robot.motorTemperatureEventLoop,
          () -> this.getDriveTemperature().gte(Constants.warnNeoOneTemp));
  private Trigger turnMotorOverheat =
      new Trigger(
          Robot.motorTemperatureEventLoop,
          () -> this.getTurnTemperature().gte(Constants.warnNeoFiveFiftyTemp));

  private Trigger driveMotorOverheatCritical =
      new Trigger(
          Robot.motorTemperatureEventLoop,
          () -> this.getDriveTemperature().gte(Constants.criticalNeoOneTemp));
  private Trigger turnMotorOverheatCritical =
      new Trigger(
          Robot.motorTemperatureEventLoop,
          () -> this.getTurnTemperature().gte(Constants.warnNeoFiveFiftyTemp));

  private Alert driveMotorOverheatAlert;
  private Alert turnMotorOverheatAlert;

  private Alert driveMotorOverheatCriticalAlert;
  private Alert turnMotorOverheatCriticalAlert;

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;
    driveDisconnectedAlert =
        new Alert(
            "Disconnected drive motor on module " + Integer.toString(index) + ".",
            AlertType.kError);
    turnDisconnectedAlert =
        new Alert(
            "Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);

    driveMotorOverheatAlert =
        new Alert(
            "MotorOverheat",
            "Overheat in module" + Integer.toString(index) + " drive motor.",
            AlertType.kWarning);
    turnMotorOverheatAlert =
        new Alert(
            "MotorOverheat",
            "Overheat in module " + Integer.toString(index) + " turn motor.",
            AlertType.kWarning);

    driveMotorOverheatAlert =
        new Alert(
            "MotorOverheat",
            "Critical Overheat in module " + Integer.toString(index) + " drive motor.",
            AlertType.kError);
    turnMotorOverheatAlert =
        new Alert(
            "MotorOverheat",
            "Critical Overheat in module " + Integer.toString(index) + " turn motor.",
            AlertType.kError);

    driveMotorOverheatCritical.onTrue(
        Commands.runOnce(() -> driveMotorOverheatCriticalAlert.set(true)));
    turnMotorOverheatCritical.onTrue(
        Commands.runOnce(() -> turnMotorOverheatCriticalAlert.set(true)));

    driveMotorOverheat.onTrue(Commands.runOnce(() -> driveMotorOverheatAlert.set(true)));
    turnMotorOverheat.onTrue(Commands.runOnce(() -> turnMotorOverheatAlert.set(true)));

    driveMotorOverheatCritical.onFalse(
        Commands.runOnce(() -> driveMotorOverheatCriticalAlert.set(false)));
    turnMotorOverheatCritical.onFalse(
        Commands.runOnce(() -> turnMotorOverheatCriticalAlert.set(false)));

    driveMotorOverheat.onFalse(Commands.runOnce(() -> driveMotorOverheatAlert.set(false)));
    turnMotorOverheat.onFalse(Commands.runOnce(() -> turnMotorOverheatAlert.set(false)));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * wheelRadiusMeters;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
  }

  /** Runs the module with the specified setpoint state. Mutates the state to optimize it. */
  public void runSetpoint(SwerveModuleState state) {
    // Optimize velocity setpoint
    state.optimize(getAngle());
    state.cosineScale(inputs.turnPosition);

    // Apply setpoints
    io.setDriveVelocity(state.speedMetersPerSecond / wheelRadiusMeters);
    io.setTurnPosition(state.angle);
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setTurnPosition(Rotation2d.kZero);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * wheelRadiusMeters;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * wheelRadiusMeters;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Returns the module velocity in rad/sec. */
  public double getFFCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }

  public Temperature getDriveTemperature() {
    return inputs.driveTemperature;
  }

  public Temperature getTurnTemperature() {
    return inputs.turnTemperature;
  }
}
