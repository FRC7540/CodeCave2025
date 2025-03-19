package frc.robot.commands.endeffector;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.EndEffectorConstants;

public class AlageIntakeCommands {
  public static Command IntakeFromGround(EndEffector endEffector) {
    return EndEffectorPresets.pickupFromGround(endEffector)
        .andThen(
            () ->
                endEffector.runEffectionVolts(
                    EndEffectorConstants.EffectionPresets.ALAGE_INTAKE_VOLTAGE),
            endEffector)
        .until(endEffector.hasBall())
        .withTimeout(Seconds.of(5.0));
  }
}
