package frc.robot.commands.climber;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.Climber;

public class ClimberCommands {
  public static Command deployClimber(Climber climber) {
    return Commands.run(() -> climber.runVolts(Volts.of(1.0)), climber);
  }

  public static Command retractClimber(Climber climber) {
    return Commands.run(() -> climber.runVolts(Volts.of(1.0)), climber);
  }
}
