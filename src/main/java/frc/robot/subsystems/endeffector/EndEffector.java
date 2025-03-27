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

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.AutoClosing;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase implements AutoClosing {
  private final EndEffectorIO endeffectorio;
  private final EndEffectorInputsAutoLogged endeffectorinputs = new EndEffectorInputsAutoLogged();
  private final SysIdRoutine sysIdRoutine;

  @AutoLogOutput(key = "EndEffector/hasBall")
  private Trigger hasBall;

  @AutoLogOutput(key = "EndEffector/hasCoral")
  private Trigger hasCoral;

  @AutoLogOutput(key = "EndEffector/clearForElevatorMotion")
  private Trigger clearForElevatorMotion;

  @AutoLogOutput(key = "EndEffector/clearForClimb")
  private Trigger clearForClimb;

  private final ArmFeedforward feedforward = new ArmFeedforward(0.38267, 0.43387, 0.50819);

  private final ProfiledPIDController feedback =
      new ProfiledPIDController(
          3.75, 0, 0.025, new TrapezoidProfile.Constraints(Math.PI, 3 * Math.PI));

  /* Should we be runnning the control system? */
  @AutoLogOutput(key = "EndEffector/controlSystemActive")
  private boolean controlSystemActive = true;

  /* Current Target Angle of the end effector */
  @AutoLogOutput(key = "EndEffector/targetAngle")
  private MutAngle targetAngle = Radians.mutable(0.0);

  private Trigger positionMotorOverheat =
      new Trigger(
          Robot.motorTemperatureEventLoop,
          () -> this.getPositionMotorTemperature().gte(Constants.warnNeoOneTemp));
  private Trigger criticalPositionMotorOverheat =
      new Trigger(
          Robot.motorTemperatureEventLoop,
          () -> this.getEffectionMotorTemperature().gte(Constants.criticalNeoOneTemp));

  private Alert positionMotorOverheatAlert =
      new Alert("MotorOverheat", "Overheat in End Effector Positonal motor.", AlertType.kWarning);
  private Alert criticalPositionMotorOverheatAlert =
      new Alert(
          "MotorOverheat", "Critical Overheat in End Effector Positonal motor.", AlertType.kError);

  private Trigger effectionMotorOverheat =
      new Trigger(
          Robot.motorTemperatureEventLoop,
          () -> this.getEffectionMotorTemperature().gte(Constants.warnNeoFiveFiftyTemp));
  private Trigger criticalEffectionMotorOverheat =
      new Trigger(
          Robot.motorTemperatureEventLoop,
          () -> getEffectionMotorTemperature().gte(Constants.criticalNeoFiveFiftyTemp));

  private Alert effectionMotorOverheatAlert =
      new Alert("MotorOverheat", "Overheat in End Effector effection motor.", AlertType.kWarning);
  private Alert criticalEfffectionMotorOverheatAlert =
      new Alert(
          "MotorOverheat", "Critical Overheat in End Effector effection motor.", AlertType.kError);

  public EndEffector(EndEffectorIO endeffectorio) {
    this.endeffectorio = endeffectorio;
    feedback.setIZone(0.08);
    feedback.setTolerance(0.0075);
    feedback.setIntegratorRange(-0.5, 0.5);

    positionMotorOverheat.onTrue(Commands.runOnce(() -> positionMotorOverheatAlert.set(true)));
    criticalPositionMotorOverheat.onTrue(
        Commands.runOnce(() -> criticalPositionMotorOverheatAlert.set(true)));

    positionMotorOverheat.onFalse(Commands.runOnce(() -> positionMotorOverheatAlert.set(false)));
    criticalPositionMotorOverheat.onFalse(
        Commands.runOnce(() -> criticalPositionMotorOverheatAlert.set(false)));

    effectionMotorOverheat.onTrue(Commands.runOnce(() -> effectionMotorOverheatAlert.set(true)));
    criticalEffectionMotorOverheat.onTrue(
        Commands.runOnce(() -> criticalEfffectionMotorOverheatAlert.set(true)));

    effectionMotorOverheat.onFalse(Commands.runOnce(() -> effectionMotorOverheatAlert.set(false)));
    criticalEffectionMotorOverheat.onFalse(
        Commands.runOnce(() -> criticalEfffectionMotorOverheatAlert.set(false)));

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.1).per(Second),
                Volts.of(0.7),
                Seconds.of(20.0), // Use default config
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> this.runPositionVolts(voltage),
                null, // No log consumer, since data is recorded by AdvantageKit
                this));
    feedback.setIntegratorRange(
        EndEffectorConstants.minControlAuthority.in(Volts),
        EndEffectorConstants.maxControlAuthority.in(Volts));
    targetAngle.mut_replace(EndEffectorConstants.maxAngle);

    hasBall =
        new Trigger(
                () -> {
                  return (endeffectorinputs.ballDetected);
                })
            .debounce(0.15, DebounceType.kFalling);

    hasCoral =
        new Trigger(
                () -> {
                  return endeffectorinputs.coralDetected;
                })
            .debounce(0.15, DebounceType.kFalling);

    // || (endeffectorinputs.effectionMotorCurrentAmps.gte(Amps.of(10.0))
    // && endeffectorinputs.effectionMotorAppliedVolts.gte(Volts.of(0.1))
    // && endeffectorinputs.effectionMotorVelocityRadPerSec.lte(
    //     RadiansPerSecond.of(2)));
    clearForElevatorMotion = new Trigger(this::goodForElevation);

    clearForClimb =
        new Trigger(
            () -> {
              return endeffectorinputs.endEffectorAbsolutePositionRad.isNear(
                  Radians.of(4.2), Radians.of(0.1));
            });
  }

  @Override
  public void periodic() {
    endeffectorio.updateInputs(endeffectorinputs);
    Logger.processInputs("EndEffector", endeffectorinputs);

    if (!this.controlSystemActive) return;

    double output =
        -1.0
            * feedback.calculate(
                endeffectorinputs.endEffectorAbsolutePositionRad.in(Radians),
                targetAngle.in(Radians));
    output +=
        feedforward.calculate(
            feedback.getSetpoint().position, -1.0 * feedback.getSetpoint().velocity);

    MathUtil.clamp(
        output,
        EndEffectorConstants.minControlAuthority.in(Volts),
        EndEffectorConstants.maxControlAuthority.in(Volts));
    endeffectorio.setPositionMotorVoltage(Volts.of(-output));
  }

  /**
   * Set the target angle of the end effector
   *
   * @param targetAngle Target Angle of the end effector
   */
  public void setTargetPosition(Angle targetAngle) {
    this.targetAngle.mut_replace(
        MathUtil.clamp(
            targetAngle.in(Radians),
            EndEffectorConstants.minAngle.in(Radians),
            EndEffectorConstants.maxAngle.in(Radians)),
        Radians);
  }

  public Angle getTargetAngle() {
    return targetAngle.copy();
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

  @AutoLogOutput(key = "Good")
  private boolean goodForElevation() {
    return getAngle().lte(Radians.of(4.2));
  }

  /**
   * Drives the end effector positonal motor at a specified voltage, used mainly for testing/tuning
   *
   * @see setControlsActive The elevator control system must be disabled for you to use this
   *     function!
   * @param voltage Elevator motor voltages
   */
  public void drivePositonalVoltage(Voltage voltage) {
    if (this.controlSystemActive == true) {
      DriverStation.reportWarning(
          "You must deactivate the end effector control systems before driving by voltage!", false);
      return;
    }
    endeffectorio.setPositionMotorVoltage(voltage);
  }

  /**
   * Disables the control system gaurd and runs at the specified voltage
   *
   * @param voltage voltage to drive motor at
   */
  public void runPositionVolts(Voltage voltage) {
    this.setControlsActive(false);
    endeffectorio.setPositionMotorVoltage(voltage);
  }

  /**
   * Runs the end effector effection motors at a desired voltage
   *
   * @param voltage The voltage to run the motor at
   */
  public void runEffectionVolts(Voltage controlVoltage) {
    endeffectorio.setEffectionOpenLoopVoltage(controlVoltage);
  }

  /**
   * Runs the end effector effection wheels at a desired velocity
   *
   * @param velocity The velocity to run the effection wheels at
   */
  public void runEffectionVelocity(AngularVelocity velocity) {
    endeffectorio.setEffectionVelocity(velocity);
  }

  /**
   * Get the current angle of the endEffector
   *
   * @return The Current Angle of the endEffector
   */
  public Angle getAngle() {
    return endeffectorinputs.endEffectorAbsolutePositionRad;
  }

  public Trigger hasBall() {
    return this.hasBall;
  }

  
  public Trigger hasCoral() {
    return this.hasCoral;
  }

  public Trigger clearForElevatorMotion() {
    return this.clearForElevatorMotion;
  }

  public Trigger clearForClimb() {
    return this.clearForElevatorMotion;
  }

  public Temperature getPositionMotorTemperature() {
    return endeffectorinputs.positionMotorTemperature;
  }

  public Temperature getEffectionMotorTemperature() {
    return endeffectorinputs.effectionMotorTemperature;
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
