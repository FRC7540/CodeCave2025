// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
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

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.climber.ClimberCommands;
import frc.robot.commands.climber.ClimberSafteyWrapper;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.drivestation.RumbleCommands;
import frc.robot.commands.elevator.ElevatorPresets;
import frc.robot.commands.elevator.ElevatorSafteyWrapper;
import frc.robot.commands.endeffector.AlageIntakeCommands;
import frc.robot.commands.endeffector.CoralIntakeCommands;
import frc.robot.commands.endeffector.EndEffectorPresets;
import frc.robot.commands.endeffector.JoystickControl;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.EmptyClimberIO;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSpark;
import frc.robot.subsystems.elevator.EmptyElevatorIO;
import frc.robot.subsystems.endeffector.EmptyEndEffectorIO;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.EndEffectorIOSim;
import frc.robot.subsystems.endeffector.EndEffectorIOSpark;
import frc.robot.subsystems.vision.EmptyVisionIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Elevator elevator;
  private final EndEffector endEffector;

  @SuppressWarnings("unused")
  private final Vision vision;

  private final Climber climber;

  private SwerveDriveSimulation driveSimulation = null;
  // Controller
  private final CommandXboxController driverController =
      new CommandXboxController(Constants.HID.ControllerInterfaces.DRIVER_CONTROLLER_ID);
  private final CommandXboxController operatorController =
      new CommandXboxController(Constants.HID.ControllerInterfaces.OPERATOR_CONTROLLER_ID);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public static final Alert unableToHomeAlert =
      new Alert(
          "Elevator homing procedure failed, Homing command was preempted", AlertType.kWarning);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));
        elevator = new Elevator(new ElevatorIOSpark());
        endEffector = new EndEffector(new EndEffectorIOSpark());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
                new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));
        climber = new Climber(new EmptyClimberIO());
        NamedCommands.registerCommand(
            "MoveArm",
            Commands.run(() -> endEffector.setTargetPosition(Radians.of(3.9)), endEffector));
        NamedCommands.registerCommand(
            "Out", Commands.run(() -> endEffector.runEffectionVolts(Volts.of(8.0)), endEffector));
        break;

      case SIM:
        // create a maple-sim swerve drive simulation instance
        this.driveSimulation =
            new SwerveDriveSimulation(
                DriveConstants.mapleSimConfig, new Pose2d(3, 3, Rotation2d.kZero));
        // add the simulated drivetrain to the simulation field
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        // Sim robot, instantiate physics sim IO implementations
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]));
        elevator = new Elevator(new ElevatorIOSim());
        endEffector = new EndEffector(new EndEffectorIOSim());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera1, drive::getPose));
        climber = new Climber(new ClimberIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        elevator = new Elevator(new EmptyElevatorIO());
        endEffector = new EndEffector(new EmptyEndEffectorIO());
        vision = new Vision(drive::addVisionMeasurement, new EmptyVisionIO(), new EmptyVisionIO());
        climber = new Climber(new EmptyClimberIO());
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();

    // Configure Subsystem HumanInteraction Automations
    endEffector.hasBall().onTrue(EndEffectorPresets.stowWithAlage(endEffector));

    endEffector
        .hasBall()
        .debounce(0.1)
        .and(operatorController.leftTrigger().negate())
        .whileTrue(Commands.run(() -> endEffector.runEffectionVolts(Volts.of(0.75)), endEffector));

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // field-relative drive at reduced speed
    driverController
        .rightBumper()
        .whileTrue(
            DriveCommands.joystickDriveWithSpeedMultiplier(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                () -> 0.1));

    // robot-relative drive, choosing slow or fast based on OI
    driverController
        .leftTrigger()
        .whileTrue(
            DriveCommands.joystickDriveRobotRelative(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX()));

    driverController
        .leftBumper()
        .whileTrue(
            DriveCommands.joystickDriveRobotRelativeWithSpeeds(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                () -> 0.5));

    // Lock to 0 when A button is held
    driverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0 when B button is pressed
    driverController
        .b()
        .debounce(1)
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .andThen(RumbleCommands.actionAccepted(driverController))
                .ignoringDisable(true));

    driverController
        .rightTrigger()
        .and(driverController.povDown())
        .whileTrue(
            ClimberSafteyWrapper.wrap(
                ClimberCommands.deployClimber(climber), elevator, endEffector));
    driverController
        .rightTrigger()
        .and(driverController.povUp())
        .whileTrue(
            ClimberSafteyWrapper.wrap(
                ClimberCommands.retractClimber(climber), elevator, endEffector));

    // Default Command, drive the manipulator using a joystick
    endEffector.setDefaultCommand(
        new JoystickControl(
            endEffector,
            operatorController::getLeftY,
            operatorController.rightTrigger(),
            operatorController.leftTrigger()));

    operatorController.a().whileTrue(AlageIntakeCommands.IntakeFromGround(endEffector));
    operatorController.x().whileTrue(AlageIntakeCommands.IntakeFromReef(endEffector));

    operatorController.b().whileTrue(CoralIntakeCommands.IntakeFromSource(endEffector));

    operatorController
        .y()
        .debounce(1)
        .whileTrue(
            ElevatorSafteyWrapper.HomeElevatorWrapped(elevator, endEffector, drive)
                .alongWith(RumbleCommands.actionAccepted(operatorController)));

    elevator.setDefaultCommand(
        ElevatorSafteyWrapper.ElevatorJoystickControlWrapped(
            operatorController::getRightY, elevator, endEffector, drive));

    /* Elevator reef controls */
    operatorController
        .rightBumper()
        .and(operatorController.povDown())
        .and(operatorController.leftBumper().negate())
        .whileTrue(
            ElevatorSafteyWrapper.wrap(ElevatorPresets.reefLevelOne(elevator), endEffector, drive));
    operatorController
        .rightBumper()
        .and(operatorController.povLeft())
        .and(operatorController.leftBumper().negate())
        .whileTrue(
            ElevatorSafteyWrapper.wrap(ElevatorPresets.reefLevelTwo(elevator), endEffector, drive));
    operatorController
        .rightBumper()
        .and(operatorController.povRight())
        .and(operatorController.leftBumper().negate())
        .whileTrue(
            ElevatorSafteyWrapper.wrap(
                ElevatorPresets.reefLevelThree(elevator), endEffector, drive));
    operatorController
        .rightBumper()
        .and(operatorController.povUp())
        .and(operatorController.leftBumper().negate())
        .whileTrue(
            ElevatorSafteyWrapper.wrap(
                ElevatorPresets.reefLevelFour(elevator), endEffector, drive));

    /* Other Elevator Controls */
    operatorController
        .leftBumper()
        .and(operatorController.povDown())
        .and(operatorController.rightBumper().negate())
        .whileTrue(ElevatorSafteyWrapper.wrap(ElevatorPresets.floor(elevator), endEffector, drive));
    operatorController
        .leftBumper()
        .and(operatorController.povUp())
        .and(operatorController.rightBumper().negate())
        .whileTrue(ElevatorSafteyWrapper.wrap(ElevatorPresets.barge(elevator), endEffector, drive));

    // Simulation Bindings
    if (Constants.currentMode == Constants.Mode.SIM) {
      configureSimulationButtonBindings();
    }
  }

  public void configureSimulationButtonBindings() {
    // L4 placement
    driverController
        .y()
        .onTrue(
            Commands.runOnce(
                () ->
                    SimulatedArena.getInstance()
                        .addGamePieceProjectile(
                            new ReefscapeCoralOnFly(
                                driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                                new Translation2d(0.4, 0),
                                driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                                Meters.of(2),
                                MetersPerSecond.of(1.5),
                                Degrees.of(-80)))));
    // L3 placement
    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                () ->
                    SimulatedArena.getInstance()
                        .addGamePieceProjectile(
                            new ReefscapeCoralOnFly(
                                driveSimulation.getSimulatedDriveTrainPose().getTranslation(),
                                new Translation2d(0.4, 0),
                                driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                                driveSimulation.getSimulatedDriveTrainPose().getRotation(),
                                Meters.of(1.35),
                                MetersPerSecond.of(1.5),
                                Degrees.of(-60)))));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return Commands.runOnce(
    //         () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
    //         drive)
    //     .andThen(
    //         DriveCommands.joystickDrive(drive, () -> -0.5, () -> 0.0, () -> 0.0)
    //             .withTimeout(Seconds.of(2.0)));

    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    drive.setPose(new Pose2d(3, 3, Rotation2d.kZero));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
