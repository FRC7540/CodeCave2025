package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;
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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.DoubleSupplier;

public class EndEffectorIOSpark implements EndEffectorIO {
  /* Hardware Objects */
  private final SparkBase positionMotor;
  private final RelativeEncoder positinMotorEncoder;
  private final AbsoluteEncoder positioinMotorEndEffectorEncoder;
  private final Debouncer positionMotorConnectedDebouncer = new Debouncer(0.5);

  private final SparkBase effectionMotor;
  private final RelativeEncoder effectionMotorEncoder;
  private final Debouncer effectionMotorConnectedDebouncer = new Debouncer(0.5);

  public EndEffectorIOSpark() {
    positionMotor = new SparkMax(EndEffectorConstants.positonalMotorCANID, MotorType.kBrushless);
    positinMotorEncoder = positionMotor.getEncoder();
    positioinMotorEndEffectorEncoder = positionMotor.getAbsoluteEncoder();

    var positionMotorConfig = new SparkMaxConfig();
    positionMotorConfig.encoder.velocityConversionFactor(
        EndEffectorConstants.positonalDriveMotorEndcoderVelocityFactor);
    positionMotorConfig.encoder.positionConversionFactor(
        EndEffectorConstants.positonalDriveMotorEndcoderPositionFactor);
    positionMotorConfig.absoluteEncoder.inverted(true);
    positionMotorConfig.absoluteEncoder.zeroOffset(
        EndEffectorConstants.positonEncoderOffset.in(Rotations));
    positionMotorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) EndEffectorConstants.positonalMotorMaxCurrent.in(Amp))
        .voltageCompensation(EndEffectorConstants.positonalMotorNominalVoltage.in(Volt));

    System.out.println("AAA:: " + EndEffectorConstants.positonalDriveMotorEndcoderVelocityFactor);
    // positionMotorConfig.encoder.uvwMeasurementPeriod(10).uvwAverageDepth(2);

    tryUntilOk(
        positionMotor,
        5,
        () ->
            positionMotor.configure(
                positionMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    effectionMotor = new SparkMax(EndEffectorConstants.effectionMotorCANID, MotorType.kBrushless);
    effectionMotorEncoder = effectionMotor.getEncoder();

    var effectionMotorConfig = new SparkMaxConfig();
    effectionMotorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) EndEffectorConstants.effectionMotorMaxCurrent.in(Amp))
        .voltageCompensation(EndEffectorConstants.effectionMotorNominalVoltage.in(Volt));

    tryUntilOk(
        effectionMotor,
        5,
        () ->
            effectionMotor.configure(
                effectionMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    positinMotorEncoder.setPosition(positioinMotorEndEffectorEncoder.getPosition());
  }

  @Override
  public void updateInputs(EndEffectorInputs inputs) {
    sparkStickyFault = false;
    ifOk(
        positionMotor,
        positinMotorEncoder::getPosition,
        (value) -> inputs.positionMotorPositionRad.mut_replace(value, Rotations));
    ifOk(
        positionMotor,
        positinMotorEncoder::getVelocity,
        (value) -> inputs.positionMotorVelocityRadPerSec.mut_replace(value, RotationsPerMinute));
    ifOk(
        positionMotor,
        new DoubleSupplier[] {positionMotor::getAppliedOutput, positionMotor::getBusVoltage},
        (values) -> inputs.positionMotorAppliedVolts.mut_replace(values[0] * values[1], Volt));
    ifOk(
        positionMotor,
        positionMotor::getOutputCurrent,
        (value) -> inputs.positionMotorCurrentAmps.mut_replace(value, Amp));
    inputs.positionMotorIsConnected = positionMotorConnectedDebouncer.calculate(!sparkStickyFault);
    ifOk(
        positionMotor,
        positioinMotorEndEffectorEncoder::getPosition,
        (value) -> inputs.endEffectorAbsolutePositionRad.mut_replace(value, Rotations));
    ifOk(
        positionMotor,
        positioinMotorEndEffectorEncoder::getVelocity,
        (value) ->
            inputs.enfEffectorAbsoluteVelocityRadPerSec.mut_replace(value, RotationsPerMinute));

    sparkStickyFault = false;
    ifOk(
        effectionMotor,
        effectionMotorEncoder::getPosition,
        (value) -> inputs.effectionMotorPositionRad.mut_replace(value, Rotations));
    ifOk(
        effectionMotor,
        effectionMotorEncoder::getVelocity,
        (value) -> inputs.effectionMotorVelocityRadPerSec.mut_replace(value, RotationsPerMinute));
    ifOk(
        effectionMotor,
        new DoubleSupplier[] {effectionMotor::getAppliedOutput, effectionMotor::getBusVoltage},
        (values) -> inputs.effectionMotorAppliedVolts.mut_replace(values[0] * values[1], Volt));
    ifOk(
        effectionMotor,
        effectionMotor::getOutputCurrent,
        (value) -> inputs.effectionMotorCurrentAmps.mut_replace(value, Amp));
    inputs.effectionMotorIsConnected =
        effectionMotorConnectedDebouncer.calculate(!sparkStickyFault);
  }

  @Override
  public void setPositionMotorVoltage(Voltage voltage) {
    /* Software motion constraints */
    if (Rotations.of(positioinMotorEndEffectorEncoder.getPosition())
        .lte(EndEffectorConstants.minAngle)) {
      voltage = Volts.of(MathUtil.clamp(voltage.in(Volts), 0.0, Double.MAX_VALUE));
    }
    if (Rotations.of(positioinMotorEndEffectorEncoder.getPosition())
        .gte(EndEffectorConstants.maxAngle)) {
      voltage = Volts.of(MathUtil.clamp(voltage.in(Volts), Double.MIN_VALUE, 0.0));
    }
    positionMotor.setVoltage(voltage);
  }

  @Override
  public void setEffectionMotorVoltage(Voltage voltage) {
    effectionMotor.setVoltage(voltage);
  }
}
