package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.ExtraUnits.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.DoubleSupplier;

public class ClimberIOSpark implements ClimberIO {

  private final SparkBase motor;
  private final RelativeEncoder motorEncoder;
  private final Debouncer motorConnectedDebouncer =
      new Debouncer(ClimberConstants.GENERAL_DEBOUNCE_TIME.in(Seconds));

  public ClimberIOSpark() {
    motor = new SparkMax(ClimberConstants.motorCANId, MotorType.kBrushless);
    motorEncoder = motor.getEncoder();

    var motorConfig = new SparkMaxConfig();
    motorConfig
        .inverted(false)
        .smartCurrentLimit(0)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12.0);
    motorConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);

    tryUntilOk(
        motor,
        5,
        () ->
            motor.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    sparkStickyFault = false;
    ifOk(
        motor,
        motorEncoder::getPosition,
        (value) -> inputs.motorPositionRad.mut_replace(value, Rotations));
    ifOk(
        motor,
        motorEncoder::getVelocity,
        (value) -> inputs.motorVelocityRadPerSec.mut_replace(value, RotationsPerMinute));
    ifOk(
        motor,
        new DoubleSupplier[] {motor::getAppliedOutput, motor::getBusVoltage},
        (values) -> inputs.motorAppliedVolts.mut_replace(values[0] * values[1], Volts));
    ifOk(
        motor,
        motor::getOutputCurrent,
        (value) -> inputs.motorCurrentAmps.mut_replace(value, Amps));
    ifOk(
        motor,
        motor::getMotorTemperature,
        (value) -> inputs.motorTemperature.mut_replace(value, Celsius));
    inputs.motorIsConnected = motorConnectedDebouncer.calculate(!sparkStickyFault);
  }

  @Override
  public void setMotorVoltage(Voltage voltage) {
    // TODO Auto-generated method stub
    ClimberIO.super.setMotorVoltage(voltage);
  }

  @Override
  public void setZero() {
    // TODO Auto-generated method stub
    ClimberIO.super.setZero();
  }
}
