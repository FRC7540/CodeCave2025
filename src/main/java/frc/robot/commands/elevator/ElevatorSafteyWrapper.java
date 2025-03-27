package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import java.util.function.DoubleSupplier;

public class ElevatorSafteyWrapper {

  public static Command wrap(Command elevatorCommand, EndEffector endEffector, Drive drive) {
    return elevatorCommand.onlyWhile(
        endEffector
            .clearForElevatorMotion()
            .and(() -> drive.getRobotLinearVelocity().lte(Constants.maxElevatorRobotLinearVelocity))
            .and(
                () ->
                    drive
                        .getRobotAngularVelocity()
                        .lte(Constants.maxElevatorRobotAngularVelocity)));
  }

  public static Command ElevatorJoystickControlWrapped(
      DoubleSupplier doubleSupplier, Elevator elevator, EndEffector endEffector, Drive drive) {
    return wrap(new ElevatorJoystickControl(elevator, doubleSupplier), endEffector, drive);
  }

  public static Command HomeElevatorWrapped(
      Elevator elevator, EndEffector endEffector, Drive drive) {
    return wrap(new HomeElevator(elevator), endEffector, drive);
  }
}
