package frc.robot.commands.endeffector;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.EndEffectorConstants;

public class AlageIntakeCommands {
  public static Command IntakeFromGround(EndEffector endEffector) {
    return EndEffectorPresets.pickupFromGround(endEffector)
        .andThen(
            Commands.run(
                () ->
                    endEffector.runEffectionVolts(
                        EndEffectorConstants.EffectionPresets.ALAGE_INTAKE_VOLTAGE),
                endEffector))
        .until(endEffector.hasBall())
        .withTimeout(Seconds.of(5.0));
  }

  public static Command IntakeFromReef(EndEffector endEffector) {
    return EndEffectorPresets.pickupFromReef(endEffector)
        .andThen(
            () ->
                endEffector.runEffectionVolts(
                    EndEffectorConstants.EffectionPresets.ALAGE_INTAKE_VOLTAGE),
            endEffector)
        .until(endEffector.hasBall())
        .withTimeout(Seconds.of(5.0));
  }
}
