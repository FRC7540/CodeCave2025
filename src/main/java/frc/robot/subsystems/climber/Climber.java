package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.AutoClosing;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase implements AutoClosing {
  private final ClimberIO climberIO;
  private final ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();
  private final SysIdRoutine sysIdRoutine;

  public Climber(ClimberIO climberIO) {
    this.climberIO = climberIO;

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(1.75),
                null, // Use default config
                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> this.runVolts(voltage),
                null, // No log consumer, since data is recorded by AdvantageKit
                this));
  }

  @Override
  public void periodic() {
    climberIO.updateInputs(climberInputs);
    Logger.processInputs("Climber", climberInputs);
  }

  public void runVolts(Voltage voltage) {
    climberIO.setMotorVoltage(voltage);
  }

  /**
   * @param direction SysIdDirection for routine
   * @return Command that runs the sysid routine requested
   */
  public Command sysIDQuasistatic(SysIdRoutine.Direction direction) {
    // The methods below return Command objects
    return sysIdRoutine.quasistatic(direction);
  }

  /**
   * @param direction SysIdDirection for routine
   * @return Command that runs the sysid routine requested
   */
  public Command sysIDDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
}
