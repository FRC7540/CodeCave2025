package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;

public class ClimberSafteyWrapper {
  public static Command wrap(Command climberCommand, Elevator elevator, EndEffector endEffector) {
    return climberCommand.onlyWhile(elevator.getClearForClimb().and(endEffector.clearForClimb()));
  }
}
