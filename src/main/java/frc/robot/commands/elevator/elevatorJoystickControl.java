package frc.robot.commands.elevator;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.DoubleSupplier;

public class ElevatorJoystickControl extends Command {
  private final Elevator elevator;
  private final DoubleSupplier joystickInputs;

  private final MutDistance extension = Meters.mutable(0.0);

  public ElevatorJoystickControl(Elevator elevator, DoubleSupplier joystickInputs) {
    this.elevator = elevator;
    this.joystickInputs = joystickInputs;
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.setControlsActive(true);
  }

  @Override
  public void execute() {
    elevator.setPosition(
        extension.mut_plus(
            MathUtil.applyDeadband(joystickInputs.getAsDouble(), 0.1)
                * Constants.HID.SensitivityMultipliers.ElevatorJoystickControlSensitivity,
            Meters));

    // elevator.runVolts(Volts.of(MathUtil.applyDeadband(joystickInputs.getAsDouble(), 0.1) * 2.5));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
