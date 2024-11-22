package frc.robot.subsystems.pivot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {

  private static Pivot pivotInstance = null;
  public final PivotIO io;
  private final SimpleMotorFeedforward ffModel;
  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  private double goalRad;

  private Pivot(PivotIO io) {

    this.io = io;
    goalRad = 0.0;

    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
      case SIM:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
      default:
        ffModel = new SimpleMotorFeedforward(0.0, 0.0);
        break;
    }
  }

  public static Pivot getInstance() {
    return pivotInstance;
  }

  public static Pivot initialize(PivotIO io) {
    if (Pivot.pivotInstance == null) {
      pivotInstance = new Pivot(io);
    }
    return pivotInstance;
  }

  public Command currDetPivot() {
    return Commands.sequence(
        Commands.waitUntil(() -> Intake.getInstance().noteNotInIntake()),
        Commands.runOnce(() -> Pivot.getInstance().setGoal(Math.PI / 6)),
        Commands.waitUntil(() -> !Intake.getInstance().noteNotInIntake()),
        Commands.runOnce(() -> Pivot.getInstance().setGoal(0)));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.setVoltage(inputs.appliedVolts);
    Logger.processInputs("Pivot", inputs);
  }

  public void runVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void setGoal(double goal) {
    io.setAngle(goal);
  }

  public void stop() {
    io.stop();
  }
}
