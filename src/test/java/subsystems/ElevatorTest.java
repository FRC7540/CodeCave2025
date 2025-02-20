package subsystems;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.EmptyElevatorIO;
import java.util.Arrays;
import java.util.Random;
import java.util.stream.DoubleStream;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;

class ElevatorTest {
  static final double DELTA = 1e-2;
  Elevator elevator;

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
    elevator = new Elevator(new EmptyElevatorIO());
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach
  void shutdown() throws Exception {
    elevator.close();
  }

  static Stream<Angle> angleStreamWrapper() {
    return new Random().doubles(25, 0.0, 250.5).boxed().map(Radians::of);
  }

  static Stream<Distance> distanceStreamWrapper() {
    return new Random().doubles(25, 0.0, 500.5).boxed().map(Centimeters::of);
  }

  static DoubleStream extensionPercentageWrapper() {
    return Arrays.stream(new Random().doubles(25, 0.0, 1.0).toArray());
  }
}
