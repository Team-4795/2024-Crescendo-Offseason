package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private Timer noteTime = new Timer();
  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private static Intake instance;
  public double intakeSpeed;
  public static double time1 = 0; // static
  public static boolean flag = false;

  private Intake(IntakeIO IO) {
    io = IO;
  }

  public static Intake getInstance() {
    return instance;
  }

  public static Intake initialize(IntakeIO io) {
    if (instance == null) {
      instance = new Intake(io);
    }
    return instance;
  }

  public void setIntakeSpeed(double speed) {
    intakeSpeed = speed;
  }

  public boolean noteNotInIntake() { // ..
    if (inputs.Amps < IntakeConstants.current) {
      noteTime.start();
      flag = true;
      return inputs.Amps < IntakeConstants.current;
    } else {
      if (flag) {
        time1 = noteTime.get();
        noteTime.stop();
        noteTime.reset();
      }
      return inputs.Amps < IntakeConstants.current;
    }
  }

  public Command intake() {
    return startEnd(() -> setIntakeSpeed(1), () -> setIntakeSpeed(0)).onlyWhile(() -> noteNotInIntake());
    // return (startEnd(() -> setIntakeSpeed(1), () -> setIntakeSpeed(0)).onlyWhile(() ->
    // noteNotInIntake())).alongWith(startEnd(() -> setIntakeSpeed(0.1), () ->
    // setIntakeSpeed(0)).onlyIf(() -> noteNotInIntake() == true).onlyWhile(() -> noteNotInIntake()
    // == true));
  }

  public Command reverse() {
    return startEnd(() -> setIntakeSpeed(-1), () -> setIntakeSpeed(0));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.setMotorSpeed(intakeSpeed);
  }
}