// Copyright 2025 FRC 7540
// http://https://github.com/FRC7540
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
  private final EndEffectorIO endeffectorio;
  private final EndEffectorInputsAutoLogged endeffectorinputs = new EndEffectorInputsAutoLogged();

  public EndEffector(EndEffectorIO endeffectorio) {
    this.endeffectorio = endeffectorio;
  }

  @Override
  public void periodic() {
    endeffectorio.updateInputs(endeffectorinputs);
    Logger.processInputs("EndEffector", endeffectorinputs);
  }

  public void setTargetPosition(double targetPosition) {}
}
