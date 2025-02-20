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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
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

  /* displacment: The current displacment of the carraige, from the home position (Fully Retracted) */
  @AutoLogOutput(key = "Elevator/displacment")
  private MutDistance displacment = Meters.mutable(0.0);

  /* groundExtension: the current distance from the ground to the middle of the elevator carraige */
  @AutoLogOutput(key = "Elevator/groundExtension")
  private MutDistance groundExtension = Meters.mutable(0.0);

  /* whether or not we have been homed */
  @AutoLogOutput(key = "Elevator/isHomed")
  private boolean isHomed = false;

  /* Control Systems */

  /* Should we be runnning the control system? */
  @AutoLogOutput(key = "Elevator/controlSystemActive")
  private boolean controlSystemActive = false;

  /* This holds a model of our gearbox */
  private final DCMotor elevatorMotorSystem =
      DCMotor.getNEO(2).withReduction(ElevatorConstants.gerboxReduction);

  /* This plant holds a model of our elevator, the system has the following properties:
   *
   * Eventually, we can replace this with a linear system id based on sysid data form the elevator itself.
   *
   * States: [Position, Velocity] (Of the elevator)
   * Inputs: [Voltage] (Input to the plant)
   * Outputs: [Position] (Current Position of the plant)
   */
  private final LinearSystem<N2, N1, N2> plant =
      LinearSystemId.createElevatorSystem(
          elevatorMotorSystem,
          ElevatorConstants.carraigeMass.in(Kilograms),
          ElevatorConstants.drumRadius.in(Meter),
          3.5);

  private final KalmanFilter<N2, N1, N2> observer =
      new KalmanFilter<>(
          Nat.N2(),
          Nat.N2(),
          plant,
          ElevatorConstants.stateCovarianceMatrix,
          ElevatorConstants.measurmentCovarianceMatrix,
          ElevatorConstants.nominalLoopTime.in(Seconds));

  private final LinearQuadraticRegulator<N2, N1, N2> controller =
      new LinearQuadraticRegulator<>(
          plant,
          ElevatorConstants.stateExcursionToleranceMatrix,
          ElevatorConstants.controlAuthorityMatrix,
          ElevatorConstants.nominalLoopTime.in(Seconds));

  private final LinearSystemLoop<N2, N1, N2> loop =
      new LinearSystemLoop<>(
          plant, controller, observer, 12.0, ElevatorConstants.nominalLoopTime.in(Seconds));

  public Elevator(ElevatorIO elevatorIO) {
    this.elevatorIO = elevatorIO;
    // Create the SysId routine
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
    controller.latencyCompensate(plant, ElevatorConstants.nominalLoopTime.in(Seconds), 0.025);

    /* Initalize Values */
    this.elevatorExtension.mut_replace(Meters.of(0.0));
    this.elevatorVelocity.mut_replace(MetersPerSecond.of(0.0));
    this.displacment.mut_replace(Meters.of(0.0));
    this.groundExtension.mut_replace(Meters.of(0.0));
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
        elevatorInputs.motorAPositionRad.timesConversionFactor(
            ElevatorConstants.extensionConversionFactor));
    elevatorVelocity.mut_replace(
        elevatorInputs.motorAVelocityRadPerSec.timesConversionFactor(
            ElevatorConstants.velocityConversionFactor));

    // Determine desired position / state

    if (!this.controlSystemActive) {
      /* We dont need to run the control loops, go ahead and return. Evrything after this should be control loops stuff */
      /* We should keep the model of the system updated... I think. */
      this.resetControlLoops();
      return;
    }

    // Run Control Loops

    /* Set Reference */
    loop.setNextR(VecBuilder.fill(0, 0));

    /* Correct kalaman state vector */
    loop.correct(
        VecBuilder.fill(elevatorExtension.in(Meters), elevatorVelocity.in(MetersPerSecond)));

    /* Update the LQR to generate new voltage commands and use the voltages to predict the next state */
    loop.predict(ElevatorConstants.nominalLoopTime.in(Seconds));

    /* Get the calculated plant input from the U vector, (1x1 matrix) */
    Voltage controlVoltage = Volts.of(loop.getU(0));

    // Apply Control Loops
    elevatorIO.setMotorVoltage(controlVoltage);
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
   * @param hieght Desired distance from ground to middle of elevator carraige
   */
  public void setPosition(Distance hieght) {
    System.out.println(
        "Method: driveGroundExtension - Not Implemented Yet! " + this.getSubsystem());
  }

  /**
   * Drives the elevator at a specicfic velocity
   *
   * @param speed Desired elevator velocity
   */
  public void setVelocity(LinearVelocity speed) {
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

  public void resetControlLoops() {
    if (!isHomed) {
      this.notHomedAlert.set(true);
      return;
    }
    this.loop.reset(
        VecBuilder.fill(
            this.elevatorInputs
                .motorAPositionRad
                .timesConversionFactor(ElevatorConstants.extensionConversionFactor)
                .in(Meters),
            this.elevatorInputs
                .motorAVelocityRadPerSec
                .timesConversionFactor(ElevatorConstants.velocityConversionFactor)
                .in(MetersPerSecond)));
  }
}
