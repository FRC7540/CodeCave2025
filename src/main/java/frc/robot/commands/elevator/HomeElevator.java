package frc.robot.commands.elevator;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.Elevator;

public class HomeElevator extends Command {
  private final Elevator elevator;

  public HomeElevator(Elevator newElevator) {
    elevator = newElevator;
    addRequirements(newElevator);
  }

  @Override
  public void initialize() {
    elevator.setControlsActive(false);
    elevator.setVelocity(MetersPerSecond.of(-0.25));
    RobotContainer.unableToHomeAlert.set(false);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    elevator.setVelocity(MetersPerSecond.of(0));
    if (!interrupted) {
      elevator.setZero();
    } else {
      RobotContainer.unableToHomeAlert.set(true);
      elevator.setHomed(false);
      return;
    }
    elevator.setHomed(true);
    RobotContainer.unableToHomeAlert.set(false);
    elevator.setControlsActive(true);
  }

  @Override
  // If the function returns true, then we end the command process.
  public boolean isFinished() {
    return elevator.getLowerLimitSwitch();
  }
}
