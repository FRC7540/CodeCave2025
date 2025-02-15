package frc.robot.commands.elevator;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class HomeElevator extends Command {
  private final Elevator elevator;
  private final Alert notHome =
      new Alert("Warning: Elevator not Homed, Homing command was preempted", AlertType.kWarning);

  public HomeElevator(Elevator newElevator) {
    elevator = newElevator;
    addRequirements(newElevator);
  }

  @Override
  public void initialize() {
    elevator.driveSpeed(MetersPerSecond.of(0.1));
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    elevator.driveSpeed(MetersPerSecond.of(0));
    if (!interrupted) {
      elevator.setZero();
    } else {
      notHome.set(true);
    }
    elevator.setControlsActive(true);
  }

  @Override
  // If the function returns true, then we end the command process.
  public boolean isFinished() {
    return elevator.getLowerLimitSwitch();
  }
}
