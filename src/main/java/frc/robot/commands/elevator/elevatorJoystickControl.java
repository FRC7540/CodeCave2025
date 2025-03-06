package frc.robot.commands.elevator;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.Elevator;
import java.util.function.DoubleSupplier;

public class elevatorJoystickControl extends Command {
  private final Elevator elevator;
  private final DoubleSupplier joystickInputs;

  public elevatorJoystickControl(Elevator elevator, DoubleSupplier joystickInputs) {
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
        elevator
            .getExtension()
            .plus(
                Meters.of(
                    joystickInputs.getAsDouble()
                        * Constants.HID.ElevatorJoystickControlSensitivity)));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
