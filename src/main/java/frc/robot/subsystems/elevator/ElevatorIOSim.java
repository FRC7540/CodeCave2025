package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.ExtraUnits.*;
import static frc.robot.util.SparkUtil.*;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  DCMotor elevatorMotorSystem = DCMotor.getNEO(2).withReduction(ElevatorConstants.gerboxReduction);

  private final LinearSystem<N2, N1, N2> elevatorplant =
      LinearSystemId.createElevatorSystem(
          elevatorMotorSystem,
          ElevatorConstants.carraigeMass.in(Kilograms),
          ElevatorConstants.drumRadius.in(Meter),
          1);

  private final ElevatorSim elevatorSim =
      new ElevatorSim(
          elevatorplant,
          elevatorMotorSystem,
          ElevatorConstants.minHieght.in(Meters),
          ElevatorConstants.maxHieght.in(Meters),
          true,
          0.0,
          1.0,
          1.0);

  // The observer fuses our encoder data and voltage inputs to reject noise.

  public ElevatorIOSim() {}

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.lowerLimitSwitch = elevatorSim.hasHitLowerLimit();
    inputs.upperLimitSwitch = elevatorSim.hasHitUpperLimit();

    double extensionPercentage =
        Elevator.calculateExtensionPercentageFromDisplacement(Meters.of(elevatorSim.getOutput(0)));

    inputs.motorAPositionRad.mut_replace(
        Elevator.calculateElevatorAngleRadians(extensionPercentage));
    inputs.motorAVelocityRadPerSec.mut_replace(
        elevatorSim.getVelocityMetersPerSecond(), RotationsPerMinute);
    inputs.motorAAppliedVolts.mut_replace(elevatorSim.getInput(0), Volts);
    inputs.motorACurrentAmps.mut_replace(elevatorSim.getCurrentDrawAmps(), Amps);
    inputs.motorAIsConnected = true;

    inputs.motorBPositionRad.mut_replace(
        Elevator.calculateElevatorAngleRadians(extensionPercentage));
    inputs.motorBVelocityRadPerSec.mut_replace(
        elevatorSim.getVelocityMetersPerSecond(), RotationsPerMinute);
    inputs.motorBAppliedVolts.mut_replace(elevatorSim.getInput(0), Volts);
    inputs.motorBCurrentAmps.mut_replace(elevatorSim.getCurrentDrawAmps(), Amps);
    inputs.motorBIsConnected = true;
  }

  @Override
  public void setMotorVoltage(Voltage voltage) {
    elevatorSim.setInputVoltage(voltage.in(Volts));
  }
}
