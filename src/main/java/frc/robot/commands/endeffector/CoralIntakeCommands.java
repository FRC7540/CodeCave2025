package frc.robot.commands.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.EndEffectorConstants;

public class CoralIntakeCommands {
  public static Command IntakeFromSource(EndEffector endEffector) {
    return EndEffectorPresets.pickupFromSource(endEffector)
        .andThen(
            Commands.run(
                () ->
                    endEffector.runEffectionVolts(
                        EndEffectorConstants.EffectionPresets.CORAL_INTAKE_VOLTAGE),
                endEffector))
        .until(endEffector.hasCoral());
  }
}
