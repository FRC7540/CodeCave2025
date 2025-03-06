package frc.robot.commands.endeffector;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.endeffector.EndEffector;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class JoystickControl extends Command {
  private final EndEffector endEffector;
  private final DoubleSupplier joystickInputsSupplier;
  private final BooleanSupplier intakeEffectionSupplier;
  private final BooleanSupplier ejectEffectionSupplier;

  public JoystickControl(
      EndEffector endEffector,
      DoubleSupplier joystickInputsSupplier,
      BooleanSupplier runEffectionSupplier,
      BooleanSupplier ejectEffectionSupplier) {
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
    endEffector.setTargetPosition(
        endEffector
            .getAngle()
            .plus(
                Radians.of(
                    joystickInputsSupplier.getAsDouble()
                        * Constants.HID.EndEffectorJoystickControlSensitivity)));
    if (intakeEffectionSupplier.getAsBoolean()) {
      endEffector.runEffectionVolts(Volts.of(2.5));
    }

    if (ejectEffectionSupplier.getAsBoolean()) {
      endEffector.runEffectionVolts(Volts.of(-2.5));
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
