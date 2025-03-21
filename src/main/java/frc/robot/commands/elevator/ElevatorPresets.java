package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.Presets;

public class ElevatorPresets {

  public static Command floor(Elevator elevator) {
    return Commands.run(() -> elevator.setPosition(Presets.floor), elevator)
        .until(() -> elevator.getExtension().isEquivalent(Presets.floor));
  }

  public static Command reefLevelOne(Elevator elevator) {
    return Commands.run(() -> elevator.setPosition(Presets.reefLevelOne), elevator)
        .until(() -> elevator.getExtension().isEquivalent(Presets.reefLevelOne));
  }

  public static Command reefLevelTwo(Elevator elevator) {
    return Commands.run(() -> elevator.setPosition(Presets.reefLevelTwo), elevator)
        .until(() -> elevator.getExtension().isEquivalent(Presets.reefLevelTwo));
  }

  public static Command reefLevelThree(Elevator elevator) {
    return Commands.run(() -> elevator.setPosition(Presets.reefLevelThree), elevator)
        .until(() -> elevator.getExtension().isEquivalent(Presets.reefLevelThree));
  }

  public static Command reefLevelFour(Elevator elevator) {
    return Commands.run(() -> elevator.setPosition(Presets.reefLevelFour), elevator)
        .until(() -> elevator.getExtension().isEquivalent(Presets.reefLevelFour));
  }

  public static Command barge(Elevator elevator) {
    return Commands.run(() -> elevator.setPosition(Presets.barge), elevator)
        .until(() -> elevator.getExtension().isEquivalent(Presets.barge));
  }
}
