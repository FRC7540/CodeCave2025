package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volt;
import static frc.robot.util.ExtraUnits.RotationsPerMinute;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.AbsoluteEncoder;
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
import frc.robot.subsystems.elevator.ElevatorConstants;
import java.util.function.DoubleSupplier;

public class EndEffectorIOSpark implements EndEffectorIO {
  /* Hardware Objects */
  private final SparkBase motor;
  private final RelativeEncoder motorEncoder;
  private final AbsoluteEncoder endEffectorEncoder;
  private final Debouncer motorConnectedDebouncer = new Debouncer(0.5);

  public EndEffectorIOSpark() {
    motor = new SparkMax(EndEffectorConstants.effectionMotorCANID, MotorType.kBrushless);
    motorEncoder = motor.getEncoder();
    endEffectorEncoder = motor.getAbsoluteEncoder();

    var motorConfig = new SparkMaxConfig();
    motorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) ElevatorConstants.elevatorMotorMaxCurrent.in(Amp))
        .voltageCompensation(ElevatorConstants.elevatorMotorNominalVoltage.in(Volt));

    motorConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);

    tryUntilOk(
        motor,
        0,
        () ->
            motor.configure(
                motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(EndEffectorInputs inputs) {
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
        (values) -> inputs.motorAppliedVolts.mut_replace(values[0] * values[1], Volt));
    ifOk(
        motor, motor::getOutputCurrent, (value) -> inputs.motorCurrentAmps.mut_replace(value, Amp));
    inputs.motorIsConnected = motorConnectedDebouncer.calculate(!sparkStickyFault);
    ifOk(
        motor,
        endEffectorEncoder::getPosition,
        (value) -> inputs.endEffectorAbsolutePositionRad.mut_replace(value, Rotations));
    ifOk(
        motor,
        endEffectorEncoder::getVelocity,
        (value) ->
            inputs.enfEffectorAbsoluteVelocityRadPerSec.mut_replace(value, RotationsPerMinute));
  }

  @Override
  public void setMotorVoltage(Voltage voltage) {
    motor.setVoltage(voltage);
  }
}
