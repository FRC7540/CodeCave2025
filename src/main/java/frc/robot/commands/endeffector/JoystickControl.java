package frc.robot.commands.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.endeffector.EndEffector;

public class JoystickControl extends Command {
  private final EndEffector endEffector;

  public JoystickControl(EndEffector endEffector) {
    this.endEffector = endEffector;
    addRequirements(endEffector);
  }

  @Override
  public void initialize() {
    // TODO Auto-generated method stub
    super.initialize();
  }

  @Override
  public void execute() {
    // TODO Auto-generated method stub
    super.execute();
  }

  @Override
  public boolean isFinished() {
    // TODO Auto-generated method stub
    return super.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    // TODO Auto-generated method stub
    super.end(interrupted);
  }
}
