package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

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
          ElevatorConstants.simPositionStdDev,
          ElevatorConstants.simVelocityStdDev);

  // The observer fuses our encoder data and voltage inputs to reject noise.

  public ElevatorIOSim() {}

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.lowerLimitSwitch = elevatorSim.hasHitLowerLimit();
    inputs.upperLimitSwitch = elevatorSim.hasHitUpperLimit();

    inputs.motorAPositionRad.mut_replace(
        Meters.of(elevatorSim.getPositionMeters())
            .timesConversionFactor(
                ElevatorConstants.extensionConversionFactor.reciprocal()));
    inputs.motorAVelocityRadPerSec.mut_replace(
        MetersPerSecond.of(elevatorSim.getVelocityMetersPerSecond())
            .timesConversionFactor(
                ElevatorConstants.velocityConversionFactor.reciprocal()));
    inputs.motorAAppliedVolts.mut_replace(elevatorSim.getInput(0), Volts);
    inputs.motorACurrentAmps.mut_replace(elevatorSim.getCurrentDrawAmps(), Amps);
    inputs.motorAIsConnected = true;

    inputs.motorBPositionRad.mut_replace(
        Meters.of(elevatorSim.getPositionMeters())
            .timesConversionFactor(
                ElevatorConstants.extensionConversionFactor.reciprocal()));
    inputs.motorBVelocityRadPerSec.mut_replace(
        MetersPerSecond.of(elevatorSim.getVelocityMetersPerSecond())
            .timesConversionFactor(
                ElevatorConstants.velocityConversionFactor.reciprocal()));
    inputs.motorBAppliedVolts.mut_replace(elevatorSim.getInput(0), Volts);
    inputs.motorBCurrentAmps.mut_replace(elevatorSim.getCurrentDrawAmps(), Amps);
    inputs.motorBIsConnected = true;
  }

  @Override
  public void setMotorVoltage(Voltage voltage) {
    elevatorSim.setInputVoltage(voltage.in(Volts));
  }
}
