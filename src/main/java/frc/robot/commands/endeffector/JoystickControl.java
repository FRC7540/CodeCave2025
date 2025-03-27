package frc.robot.commands.endeffector;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.EndEffectorConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class JoystickControl extends Command {
  private final EndEffector endEffector;
  private final DoubleSupplier joystickInputsSupplier;
  private final Trigger intakeEffectionSupplier;
  private final Trigger ejectEffectionSupplier;

  @AutoLogOutput(key = "targetAngle")
  private MutAngle targetAngle = Radians.mutable(0.0);

  public JoystickControl(
      EndEffector endEffector,
      DoubleSupplier joystickInputsSupplier,
      Trigger runEffectionSupplier,
      Trigger ejectEffectionSupplier) {
    this.endEffector = endEffector;
    this.joystickInputsSupplier = joystickInputsSupplier;
    this.intakeEffectionSupplier = runEffectionSupplier;
    this.ejectEffectionSupplier = ejectEffectionSupplier;
    addRequirements(endEffector);
  }

  @Override
  public void initialize() {
    endEffector.setControlsActive(true);
    targetAngle.mut_replace(endEffector.getAngle());
  }

  @Override
  public void execute() {
    endEffector.setControlsActive(true);

    endEffector.setTargetPosition(
        endEffector
            .getTargetAngle()
            .plus(
                Radians.of(
                    -1.0
                        * MathUtil.applyDeadband(joystickInputsSupplier.getAsDouble(), 0.1)
                        * Constants.HID
                            .SensitivityMultipliers
                            .EndEffectorJoystickControlSensitivity)));

    if (endEffector.hasCoral().and(ejectEffectionSupplier).getAsBoolean()) {
      endEffector.runEffectionVolts(EndEffectorConstants.EffectionPresets.CORAL_EJECTION_VOLTAGE);
      return;
    }

    if (endEffector
            .getAngle()
            .isNear(EndEffectorConstants.AnglePresets.SOURCE_PICKUP, Degrees.of(10.0))
        && intakeEffectionSupplier.getAsBoolean()) {
      endEffector.runEffectionVolts(EndEffectorConstants.EffectionPresets.CORAL_INTAKE_VOLTAGE);
      return;
    }

    if (endEffector.hasCoral().and(intakeEffectionSupplier).getAsBoolean()) {
      endEffector.runEffectionVolts(EndEffectorConstants.EffectionPresets.CORAL_INTAKE_VOLTAGE);
      return;
    }

    if (intakeEffectionSupplier.getAsBoolean()) {
      endEffector.runEffectionVolts(EndEffectorConstants.EffectionPresets.ALAGE_INTAKE_VOLTAGE);
      return;
    } else if (ejectEffectionSupplier.getAsBoolean()) {
      endEffector.runEffectionVolts(EndEffectorConstants.EffectionPresets.ALAGE_EJECTION_VOLTAGE);
      return;
    }

    if (endEffector.hasBall().getAsBoolean()) {
      endEffector.runEffectionVolts(EndEffectorConstants.EffectionPresets.ALAGE_MAINTAIN_VOLTAGE);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
