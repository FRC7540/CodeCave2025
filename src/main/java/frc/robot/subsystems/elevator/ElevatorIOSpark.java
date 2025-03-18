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
import static frc.robot.util.ExtraUnits.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import java.util.function.DoubleSupplier;

/**
 * Elevator IO implementation for Spark Max motor controller and Two Limit Switches conncected to
 * the RoboRio.
 */
public class ElevatorIOSpark implements ElevatorIO {
  /* Hardware Objects  */
  private final SparkBase motorA;
  private final SparkBase motorB;
  private final RelativeEncoder motorAEncoder;
  private final RelativeEncoder motorBEncoder;
  private final Debouncer motorAConnectedDebounce = new Debouncer(0.5);
  private final Debouncer motorBConnectedDebounce = new Debouncer(0.5);
  private final DigitalInput lowerLimitSwitch;
  private final DigitalInput upperLimitSwitch;
  private final Debouncer upperLimitDebouncer;
  private final Debouncer lowerLimitDebouncer;

  public ElevatorIOSpark() {
    motorA = new SparkFlex(ElevatorConstants.motorACANID, MotorType.kBrushless);
    motorAEncoder = motorA.getEncoder();

    var motorAConfig = new SparkMaxConfig();
    motorAConfig.inverted(false);
    motorAConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) ElevatorConstants.elevatorMotorMaxCurrent.in(Amp))
        .voltageCompensation(ElevatorConstants.elevatorMotorNominalVoltage.in(Volt));
    motorAConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);

    tryUntilOk(
        motorA,
        5,
        () ->
            motorA.configure(
                motorAConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    motorB = new SparkMax(ElevatorConstants.motorBCANDID, MotorType.kBrushless);
    motorBEncoder = motorB.getEncoder();

    var motorBConfig = new SparkMaxConfig();
    motorBConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) ElevatorConstants.elevatorMotorMaxCurrent.in(Amp))
        .voltageCompensation(ElevatorConstants.elevatorMotorNominalVoltage.in(Volt));
    motorBConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);
    motorBConfig.follow(motorA, ElevatorConstants.invertMotorB);

    tryUntilOk(
        motorB,
        5,
        () ->
            motorB.configure(
                motorBConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // Configure Limit switches
    lowerLimitSwitch = new DigitalInput(ElevatorConstants.lowerLimitDIOID);
    upperLimitSwitch = new DigitalInput(ElevatorConstants.upperLimitDIOID);
    upperLimitDebouncer = new Debouncer(ElevatorConstants.limitSwitchDebounceTime.in(Seconds));
    lowerLimitDebouncer = new Debouncer(ElevatorConstants.limitSwitchDebounceTime.in(Seconds));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Always start by clearing our fault tracker
    // And now we get the data from motorcontroller A
    // This is based off of the template code from the Advantage Kit template

    /* Update values from Motor A */
    sparkStickyFault = false;
    ifOk(
        motorA,
        motorAEncoder::getPosition,
        (value) -> inputs.motorAPositionRad.mut_replace(value, Rotations));
    ifOk(
        motorA,
        motorAEncoder::getVelocity,
        (value) -> inputs.motorAVelocityRadPerSec.mut_replace(value, RotationsPerMinute));
    ifOk(
        motorA,
        new DoubleSupplier[] {motorA::getAppliedOutput, motorA::getBusVoltage},
        (values) -> inputs.motorAAppliedVolts.mut_replace(values[0] * values[1], Volts));
    ifOk(
        motorA,
        motorA::getOutputCurrent,
        (value) -> inputs.motorACurrentAmps.mut_replace(value, Amps));
    inputs.motorAIsConnected = motorAConnectedDebounce.calculate(!sparkStickyFault);

    /* Update Values from Motor B */
    sparkStickyFault = false;
    ifOk(
        motorB,
        motorBEncoder::getPosition,
        (value) -> inputs.motorBPositionRad.mut_replace(value, Rotations));
    ifOk(
        motorB,
        motorBEncoder::getVelocity,
        (value) -> inputs.motorBVelocityRadPerSec.mut_replace(value, RotationsPerMinute));
    ifOk(
        motorB,
        new DoubleSupplier[] {motorB::getAppliedOutput, motorB::getBusVoltage},
        (values) -> inputs.motorBAppliedVolts.mut_replace(values[0] * values[1], Volts));
    ifOk(
        motorB,
        motorB::getOutputCurrent,
        (value) -> inputs.motorBCurrentAmps.mut_replace(value, Amps));
    inputs.motorBIsConnected = motorBConnectedDebounce.calculate(!sparkStickyFault);

    /* Update values from limit switches */
    inputs.lowerLimitSwitch = lowerLimitDebouncer.calculate(lowerLimitSwitch.get());
    inputs.upperLimitSwitch = upperLimitDebouncer.calculate(upperLimitSwitch.get());
  }

  @Override
  public void setMotorVoltage(Voltage voltage) {
    /* Apply motor hardstops */
    if (lowerLimitDebouncer.calculate(lowerLimitSwitch.get())) {
      voltage = Volts.of(MathUtil.clamp(voltage.in(Volts), 0.0, Double.MAX_VALUE));
    }
    if (upperLimitDebouncer.calculate(upperLimitSwitch.get())) {
      voltage = Volts.of(MathUtil.clamp(voltage.in(Volts), Double.MIN_VALUE, 0.0));
    }

    motorA.setVoltage(voltage);
  }

  @Override
  public void setZero() {
    motorAEncoder.setPosition(0);
    motorBEncoder.setPosition(0);
  }
}
