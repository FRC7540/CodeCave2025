package frc.robot.commands.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.EndEffectorConstants;

public class EndEffectorPresets {

  public static Command pickupFromGround(EndEffector endEffector) {
    return Commands.run(
            () -> endEffector.setTargetPosition(EndEffectorConstants.AnglePresets.GROUND_PICKUP),
            endEffector)
        .until(
            () ->
                endEffector
                    .getAngle()
                    .isEquivalent(EndEffectorConstants.AnglePresets.GROUND_PICKUP));
  }

  public static Command pickupFromReef(EndEffector endEffector) {
    return Commands.run(
            () -> endEffector.setTargetPosition(EndEffectorConstants.AnglePresets.REEF_PICKUP),
            endEffector)
        .until(
            () ->
                endEffector.getAngle().isEquivalent(EndEffectorConstants.AnglePresets.REEF_PICKUP));
  }

  public static Command stowWithAlage(EndEffector endEffector) {
    return Commands.run(
            () -> endEffector.setTargetPosition(EndEffectorConstants.AnglePresets.STOW_WITH_ALAGE),
            endEffector)
        .until(
            () ->
                endEffector
                    .getAngle()
                    .isEquivalent(EndEffectorConstants.AnglePresets.STOW_WITH_ALAGE));
  }

  public static Command stowWithoutAlage(EndEffector endEffector) {
    return Commands.run(
            () ->
                endEffector.setTargetPosition(EndEffectorConstants.AnglePresets.STOW_WITHOUT_ALAGE),
            endEffector)
        .until(
            () ->
                endEffector
                    .getAngle()
                    .isEquivalent(EndEffectorConstants.AnglePresets.STOW_WITHOUT_ALAGE));
  }
}
