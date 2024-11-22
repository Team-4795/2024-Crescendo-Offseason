package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class Auto {
  public static Command taxiandshoot() {
    return Commands.sequence(
        DriveCommands.joystickDrive(Drive.getInstance(), () -> .5, () -> 0, () -> 0).withTimeout(3),
        DriveCommands.joystickDrive(Drive.getInstance(), () -> 0, () -> 0, () -> 0),
        Commands.race(
            Commands.startEnd(
                () -> Shooter.getInstance().setVoltage(9),
                () -> Shooter.getInstance().setVoltage(0)),
            Commands.sequence(
                Commands.waitSeconds(3),
                Commands.startEnd(
                        () -> Intake.getInstance().setIntakeSpeed(-0.75),
                        () -> Intake.getInstance().setIntakeSpeed(0))
                    .withTimeout(3))));
  }
}
