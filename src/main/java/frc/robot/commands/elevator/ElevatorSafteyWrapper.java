package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import java.util.function.DoubleSupplier;

public class ElevatorSafteyWrapper {

  public static Command wrap(Command elevatorCommand, EndEffector endEffector) {
    return elevatorCommand.onlyWhile(endEffector.clearForElevatorMotion());
  }

  public static Command ElevatorJoystickControlWrapped(
      DoubleSupplier doubleSupplier, Elevator elevator, EndEffector endEffector) {
    return wrap(new ElevatorJoystickControl(elevator, doubleSupplier), endEffector);
  }

  public static Command HomeElevatorWrapped(Elevator elevator, EndEffector endEffector) {
    return wrap(new HomeElevator(elevator), endEffector);
  }
}
