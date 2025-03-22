package frc.robot.commands.drivestation;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Constants;

public class RumbleCommands {
  // Tehcnically theese commands could end up having multiple things try to set the rumble at the
  // same time.
  // I could remove the schedule command but then they would block the subsystem for time just to do
  // rumble.
  // It should be fine if multiple things end up changing the rumble, its not worth the extra
  // mutexed syubsytem time.

  /** Two quick nudges on the given joystick */
  public static Command actionAccepted(CommandGenericHID joystick) {
    return new ScheduleCommand(
        Commands.runOnce(
                () -> {
                  joystick.setRumble(
                      RumbleType.kBothRumble, Constants.HID.RumbleStrengths.NUDGE_RUMBLE);
                })
            .andThen(Commands.waitSeconds(0.1))
            .andThen(
                Commands.runOnce(
                    () -> {
                      joystick.setRumble(RumbleType.kBothRumble, 0.0);
                    }))
            .andThen(Commands.waitSeconds(0.25))
            .andThen(
                Commands.runOnce(
                    () -> {
                      joystick.setRumble(
                          RumbleType.kBothRumble, Constants.HID.RumbleStrengths.NUDGE_RUMBLE);
                    }))
            .andThen(Commands.waitSeconds(0.1))
            .andThen(
                Commands.runOnce(
                    () -> {
                      joystick.setRumble(RumbleType.kBothRumble, 0.0);
                    })));
  }
  ;
}
