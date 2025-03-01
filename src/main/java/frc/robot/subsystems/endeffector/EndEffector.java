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

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
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

  /* This holds a model of our gearbox */
  private final DCMotor positonalMotorSystem =
      DCMotor.getNEO(2).withReduction(EndEffectorConstants.positonalDriveGearing);
  /* This plant holds a model of our elevator, the system has the following properties:
   *
   * Eventually, we can replace this with a linear system id based on sysid data form the elevator itself.
   *
   * States: [Position, Velocity] (Of the elevator)
   * Inputs: [Voltage] (Input to the plant)
   * Outputs: [Position] (Current Position of the plant)
   */
  private final LinearSystem<N2, N1, N2> plant =
      LinearSystemId.createSingleJointedArmSystem(
          positonalMotorSystem,
          EndEffectorConstants.mechanismMOI.in(KilogramSquareMeters),
          EndEffectorConstants.positonalDriveGearing);

  private final KalmanFilter<N2, N1, N2> observer =
      new KalmanFilter<>(
          Nat.N2(),
          Nat.N2(),
          plant,
          EndEffectorConstants.stateCovarianceMatrix,
          EndEffectorConstants.measurmentCovarianceMatrix,
          EndEffectorConstants.nominalLoopTime.in(Seconds));

  private final LinearQuadraticRegulator<N2, N1, N2> controller =
      new LinearQuadraticRegulator<>(
          plant,
          EndEffectorConstants.stateExcursionToleranceMatrix,
          EndEffectorConstants.controlAuthorityMatrix,
          EndEffectorConstants.nominalLoopTime.in(Seconds));

  private final LinearSystemLoop<N2, N1, N2> loop =
      new LinearSystemLoop<>(
          plant, controller, observer, 12.0, EndEffectorConstants.nominalLoopTime.in(Seconds));

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

    /* Configure LQR */
    if (Robot.isReal()) {
      controller.latencyCompensate(plant, EndEffectorConstants.nominalLoopTime.in(Seconds), 0.025);
    }
  }

  @Override
  public void periodic() {
    endeffectorio.updateInputs(endeffectorinputs);
    Logger.processInputs("EndEffector", endeffectorinputs);

    /* Determine What to feed the control loops */

    /* Feed the loops */
    if (!controlSystemActive) {
      this.resetControlLoops();
      return;
    }

    /* Set the next Reference */
    loop.setNextR(VecBuilder.fill(0, 0));

    /* Correct the system state model */
    loop.correct(
        VecBuilder.fill(
            endeffectorinputs.endEffectorAbsolutePositionRad.in(Radians),
            endeffectorinputs.enfEffectorAbsoluteVelocityRadPerSec.in(RadiansPerSecond)));

    loop.predict(EndEffectorConstants.nominalLoopTime.in(Seconds));

    Voltage controlVoltage = Volts.of(loop.getU(0));

    /* Apply the values */
    endeffectorio.setMotorVoltage(controlVoltage);
  }

  public void resetControlLoops() {
    loop.reset(
        VecBuilder.fill(
            endeffectorinputs.endEffectorAbsolutePositionRad.in(Radians),
            endeffectorinputs.enfEffectorAbsoluteVelocityRadPerSec.in(RadiansPerSecond)));
  }

  public void setTargetPosition(Angle targetAngle) {}

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
