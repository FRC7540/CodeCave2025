package frc.robot.commands.endeffector;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.endeffector.EndEffector;
import java.util.function.DoubleSupplier;

public class JoystickControl extends Command {
  private final EndEffector endEffector;
  private final DoubleSupplier joystickInputsSupplier;
  private final Trigger intakeEffectionSupplier;
  private final Trigger ejectEffectionSupplier;

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
  }

  @Override
  public void execute() {
    endEffector.setControlsActive(true);
    endEffector.setTargetPosition(
        endEffector
            .getAngle()
            .plus(
                Radians.of(
                    -1.0
                        * MathUtil.applyDeadband(joystickInputsSupplier.getAsDouble(), 0.1)
                        * Constants.HID.EndEffectorJoystickControlSensitivity)));
    if (intakeEffectionSupplier.getAsBoolean()) {
      endEffector.runEffectionVolts(Volts.of(6.0));
      return;
    } else {
      endEffector.runEffectionVolts(Volts.of(0.));
    }

    if (ejectEffectionSupplier.getAsBoolean()) {
      endEffector.runEffectionVolts(Volts.of(-8.0));
      return;
    } else {
      endEffector.runEffectionVolts(Volts.of(0.0));
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
