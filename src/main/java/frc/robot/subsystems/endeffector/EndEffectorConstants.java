package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public class EndEffectorConstants {

  /* Hardware Definitons */
  public static final int effectionMotorCANID = 12;
  public static final Current effectionMotorMaxCurrent = Amps.of(15);
  public static final Voltage effectionMotorNominalVoltage = Volts.of(12.0);

  public static final int positonalMotorCANID = 13;
  public static final Current positonalMotorMaxCurrent = Amps.of(50);
  public static final Voltage positonalMotorNominalVoltage = Volts.of(12.0);

  /* Motion Definitions */
  public static final double encoderPositionFactor = 1.0;
  public static final double encoderVelocityFactor = 1.0;
  public static final double positonalDriveGearing = (26.0 / 14.0) * 12.0;
  public static final double positonalDriveMotorEndcoderVelocityFactor = 1 / positonalDriveGearing;
  public static final double positonalDriveMotorEndcoderPositionFactor = 1 / positonalDriveGearing;
  public static final Angle positonEncoderOffset =
      Radians.of(Math.PI - 2.75); // (2.75 - 2.50) + Math.PI

  public static final MomentOfInertia mechanismMOI = KilogramSquareMeters.of(0.075);

  // Angle endstops
  public static final Angle minAngle = Radians.of(2.2);
  public static final Angle maxAngle = Radians.of(4.4);

  public static final Angle minElevatorClearedAngle = Radians.of(2.5);
  public static final Angle maxElevatorClearedAngle = Radians.of(4.0);

  public static final Angle climbClearedAngle = Radians.of(2.5);

  public static final Angle clearForClimbTolerance = Degrees.of(5);

  public static final Time DEBOUNCE_TIME = Seconds.of(0.5);

  public class AnglePresets {
    public static final Angle GROUND_PICKUP = Radians.of(3.34);
    public static final Angle STOW_WITH_ALAGE = Radians.of(4.0);
    public static final Angle REEF_PICKUP = Radians.of(3.14);
    public static final Angle STOW_WITHOUT_ALAGE = Radians.of(4.0);
    public static final Angle SOURCE_PICKUP = Radians.of(3.75);
  }

  public class EffectionPresets {
    public static final Voltage ALAGE_INTAKE_VOLTAGE = Volts.of(6.0);
    public static final Voltage ALAGE_EJECTION_VOLTAGE = Volts.of(-8.0);
    public static final Voltage ALAGE_MAINTAIN_VOLTAGE = Volts.of(0.75);
    public static final Voltage CORAL_INTAKE_VOLTAGE = Volts.of(-3.5);
    public static final Voltage CORAL_EJECTION_VOLTAGE = Volts.of(5.0);
  }

  public class EffectionTuning {
    public static final double kS = 0.0;
    public static final double kV = 0.1;
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final AngularAcceleration MAX_ACCELERATION = RadiansPerSecondPerSecond.of(25.0);
  }

  public class EffectionHardwareDefinitions {
    public static final double encoderVelocityConversionFactor = 4.0;
    public static final double encoderPositionConversionFactor = 4.0;
  }

  /* Control System Definitions */
  public static final Time NOMINAL_LOOP_TIME = Milliseconds.of(20);
  public static final Voltage minControlAuthority = Volts.of(8.0);
  public static final Voltage maxControlAuthority = Volts.of(-4.0);

  /* Derived Constants */
}
