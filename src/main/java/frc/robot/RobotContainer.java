// Copyright 2021-2024 FRC 6328
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

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOReal;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOReal;
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
  private final Intake intake;
  private final Pivot pivot;
  private final Shooter shooter;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController controller1 = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavX(),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        intake = Intake.initialize(new IntakeIOReal());
        pivot = Pivot.initialize(new PivotIOReal());
        shooter = Shooter.initialize(new ShooterIOReal());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        intake = Intake.initialize(new IntakeIOSim());
        pivot = Pivot.initialize(new PivotIOSim());
        shooter = Shooter.initialize(new ShooterIOReal());
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
        intake = Intake.initialize(new IntakeIO() {});
        pivot = Pivot.initialize(new PivotIO() {});
        shooter = Shooter.initialize(new ShooterIOReal());
        // change?
        break;
    }

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
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
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    controller1
        .rightTrigger()
        .whileTrue(
            Commands.startEnd(() -> intake.setIntakeSpeed(0.75), () -> intake.setIntakeSpeed(0)));

    controller1
        .leftTrigger()
        .whileTrue(
            Commands.startEnd(() -> intake.setIntakeSpeed(-0.75), () -> intake.setIntakeSpeed(0)));

    controller1
        .leftBumper()
        .whileTrue(Commands.startEnd(() -> pivot.runVoltage(3), () -> pivot.runVoltage(0), pivot));

    controller1
        .rightBumper()
        .whileTrue(Commands.startEnd(() -> pivot.runVoltage(-3), () -> pivot.runVoltage(0), pivot));

    // Shoot sequence
    controller
        .leftTrigger()
        .onTrue(
            Commands.race(
                Commands.startEnd(
                    () -> shooter.setLeftRightVoltage(11, 2), () -> shooter.setVoltage(0)),
                Commands.sequence(
                    Commands.waitSeconds(2),
                    Commands.startEnd(
                            () -> intake.setIntakeSpeed(0.75), () -> intake.setIntakeSpeed(0))
                        .withTimeout(1))));

    // controller
    //     .povLeft()
    //     .onTrue(
    //         Commands.sequence(
    //             Commands.startEnd(() -> pivot.runVoltage(3), () -> pivot.runVoltage(0), pivot)
    //                 .withTimeout(1),
    //             Commands.race(
    //                 Commands.startEnd(() -> shooter.setVoltage(9), () -> shooter.setVoltage(0)),
    //                 Commands.sequence(
    //                     Commands.waitSeconds(3),
    //                     Commands.startEnd(
    //                             () -> intake.setIntakeSpeed(-0.75), () ->
    // intake.setIntakeSpeed(0))
    //                         .withTimeout(3)))));

    controller
        .povDown()
        .whileTrue(
            Commands.startEnd(() -> shooter.setVoltage(-9), () -> shooter.setVoltage(0), pivot));

    controller
        .povRight()
        .whileTrue(
            Commands.startEnd(
                () -> shooter.setLeftRightVoltage(11, 2), () -> shooter.setLeftRightVoltage(0, 0)));

    controller.a().whileTrue(Commands.runOnce(() -> drive.zeroHeading(), drive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
