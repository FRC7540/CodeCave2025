package subsystems;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.EmptyElevatorIO;
import java.util.Arrays;
import java.util.Random;
import java.util.stream.DoubleStream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

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

  @ParameterizedTest
  @MethodSource("doubleStreamWrapper")
  void calculateExtensionPercentageBoundsCheck(double argument) {
    assert 0.0 <= elevator.calculateExtensionPercentage(argument);
    assert elevator.calculateExtensionPercentage(argument) <= 1.0;
  }

  @ParameterizedTest
  @MethodSource("doubleStreamWrapper")
  void calculateExtensionPercentageFromGroundExtensionBoundsCheck(double argument) {
    try {
      assert 0.0
          <= elevator.calculateExtensionPercentageFromGroundExtension(Centimeters.of(argument));
      assert elevator.calculateExtensionPercentageFromGroundExtension(Centimeters.of(argument))
          <= 1.0;
    } catch (java.util.ConcurrentModificationException e) {

    }
  }

  @ParameterizedTest
  @MethodSource("doubleStreamWrapper")
  void calculateExtensionPercentageFromDisplacementBoundsCheck(double argument) {
    try {
      assert 0.0 <= elevator.calculateExtensionPercentageFromDisplacement(Centimeters.of(argument));
      assert elevator.calculateExtensionPercentageFromDisplacement(Centimeters.of(argument)) <= 1.0;
    } catch (java.util.ConcurrentModificationException e) {
    }
  }

  @ParameterizedTest
  @MethodSource("extensionPercentageWrapper")
  void calculateDisplacementBoundsCheck(double argument) {
    try {
      assert 0.0 <= elevator.calculateDisplacement(argument).in(Meters);
      assert elevator.calculateDisplacement(argument).in(Meters) <= 10.0;
    } catch (java.util.ConcurrentModificationException e) {
    }
  }

  @ParameterizedTest
  @MethodSource("extensionPercentageWrapper")
  void calculateGroundExtensionBoundsCheck(double argument) {
    try {
      assert 0.0 <= elevator.calculateGroundExtension(argument).in(Meters);
      assert elevator.calculateGroundExtension(argument).in(Meters) <= 10.0;
    } catch (java.util.ConcurrentModificationException e) {
    }
  }

  @ParameterizedTest
  @MethodSource("doubleStreamWrapper")
  void elevatorCalculationUnityTest(double argument) {
    double extensionPercentage = elevator.calculateExtensionPercentage(argument);
    Distance groundExtension = elevator.calculateGroundExtension(extensionPercentage);
    Distance displacment = elevator.calculateDisplacement(extensionPercentage);

    double fromground = elevator.calculateExtensionPercentageFromGroundExtension(groundExtension);
    double fromdisplacment = elevator.calculateExtensionPercentageFromDisplacement(displacment);

    assertEquals(extensionPercentage, fromground, DELTA);
    assertEquals(extensionPercentage, fromdisplacment, DELTA);
  }

  static DoubleStream doubleStreamWrapper() {
    return Arrays.stream(new Random().doubles(25, 0.0, 250.5).toArray());
  }

  static DoubleStream extensionPercentageWrapper() {
    return Arrays.stream(new Random().doubles(25, 0.0, 1.0).toArray());
  }
}
