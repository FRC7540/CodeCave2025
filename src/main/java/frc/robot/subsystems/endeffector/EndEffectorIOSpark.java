package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volt;
import static frc.robot.util.ExtraUnits.RotationsPerMinute;
import static frc.robot.util.SparkUtil.ifOk;
import static frc.robot.util.SparkUtil.sparkStickyFault;
import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import java.util.function.DoubleSupplier;

public class EndEffectorIOSpark implements EndEffectorIO {
  /* Hardware Objects */
  private final SparkBase positionMotor;
  private final RelativeEncoder positinMotorEncoder;
  private final AbsoluteEncoder positioinMotorEndEffectorEncoder;
  private final Debouncer positionMotorConnectedDebouncer =
      new Debouncer(EndEffectorConstants.DEBOUNCE_TIME.in(Seconds));

  private final SparkBase effectionMotor;
  private final RelativeEncoder effectionMotorEncoder;
  private final Debouncer effectionMotorConnectedDebouncer =
      new Debouncer(EndEffectorConstants.DEBOUNCE_TIME.in(Seconds));

  private final SparkClosedLoopController effectionController;

  private final DigitalInput ballDetection = new DigitalInput(2);
  private final Debouncer ballDetectionDebouncer = new Debouncer(0.25);

  private final LinearFilter vfilt =
      LinearFilter.singlePoleIIR(
          Milliseconds.of(75).in(Seconds), EndEffectorConstants.NOMINAL_LOOP_TIME.in(Seconds));

  private final LinearFilter pfilt =
      LinearFilter.singlePoleIIR(
          Milliseconds.of(75).in(Seconds), EndEffectorConstants.NOMINAL_LOOP_TIME.in(Seconds));

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
    positionMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    positionMotorConfig
        .softLimit
        .forwardSoftLimit(EndEffectorConstants.maxAngle.in(Rotations))
        .forwardSoftLimitEnabled(true);
    positionMotorConfig
        .softLimit
        .reverseSoftLimit(EndEffectorConstants.minAngle.in(Rotations))
        .reverseSoftLimitEnabled(true);
    positionMotorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) EndEffectorConstants.positonalMotorMaxCurrent.in(Amp))
        .voltageCompensation(EndEffectorConstants.positonalMotorNominalVoltage.in(Volt));

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
    effectionController = effectionMotor.getClosedLoopController();

    var effectionMotorConfig = new SparkMaxConfig();
    effectionMotorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit((int) EndEffectorConstants.effectionMotorMaxCurrent.in(Amp))
        .voltageCompensation(EndEffectorConstants.effectionMotorNominalVoltage.in(Volt));
    effectionMotorConfig
        .encoder
        .positionConversionFactor(
            EndEffectorConstants.EffectionHardwareDefinitions.encoderPositionConversionFactor)
        .velocityConversionFactor(
            EndEffectorConstants.EffectionHardwareDefinitions.encoderPositionConversionFactor)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);
    effectionMotorConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pidf(
            EndEffectorConstants.EffectionTuning.kP,
            EndEffectorConstants.EffectionTuning.kI,
            EndEffectorConstants.EffectionTuning.kD,
            0.0)
        .maxMotion
        .maxAcceleration(
            EndEffectorConstants.EffectionTuning.MAX_ACCELERATION.in(RotationsPerSecondPerSecond));

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
    ifOk(
        positionMotor,
        positionMotor::getMotorTemperature,
        (value) -> inputs.positionMotorTemperature.mut_replace(value, Celsius));
    ifOk(
        positionMotor,
        positioinMotorEndEffectorEncoder::getPosition,
        (value) ->
            inputs.endEffectorAbsolutePositionRad.mut_replace(pfilt.calculate(value), Rotations));
    ifOk(
        positionMotor,
        positioinMotorEndEffectorEncoder::getVelocity,
        (value) ->
            inputs.endEffectorAbsoluteVelocityRadPerSec.mut_replace(
                vfilt.calculate(value), RotationsPerMinute));
    inputs.positionMotorIsConnected = positionMotorConnectedDebouncer.calculate(!sparkStickyFault);

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
    ifOk(
        effectionMotor,
        effectionMotor::getMotorTemperature,
        (value) -> inputs.effectionMotorTemperature.mut_replace(value, Celsius));
    inputs.effectionMotorIsConnected =
        effectionMotorConnectedDebouncer.calculate(!sparkStickyFault);

    inputs.ballDetected = ballDetectionDebouncer.calculate(ballDetection.get());
  }

  @Override
  public void setPositionMotorVoltage(Voltage voltage) {
    /* Software motion constraints */
    // if (Rotations.of(positioinMotorEndEffectorEncoder.getPosition())
    //     .lte(EndEffectorConstants.minAngle)) {
    //   voltage = Volts.of(MathUtil.clamp(voltage.in(Volts), 0.0, Double.MAX_VALUE));
    // }
    // if (Rotations.of(positioinMotorEndEffectorEncoder.getPosition())
    //     .gte(EndEffectorConstants.maxAngle)) {
    //   voltage = Volts.of(MathUtil.clamp(voltage.in(Volts), Double.MIN_VALUE, 0.0));
    // }
    positionMotor.setVoltage(voltage);
  }

  @Override
  public void setEffectionOpenLoopVoltage(Voltage voltage) {
    effectionMotor.setVoltage(voltage);
  }

  @Override
  public void setEffectionVelocity(AngularVelocity velocity) {
    double ffVolts =
        EndEffectorConstants.EffectionTuning.kS * Math.signum(velocity.in(RadiansPerSecond))
            + EndEffectorConstants.EffectionTuning.kV * velocity.in(RadiansPerSecond);
    effectionController.setReference(
        velocity.in(RotationsPerMinute),
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ffVolts,
        ArbFFUnits.kVoltage);
  }
}
