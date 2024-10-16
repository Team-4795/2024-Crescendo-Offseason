package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  @AutoLog
  public static class PivotIOInputs {
    public double positionRad = 0.0;
    public double appliedVolts = 0.0;
    public double velocityRadPerSec = 0.0;
    public double[] currentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(PivotIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Run closed loop at the specified velocity. */
  public default void setAngle(double positionRad) {}

  /** Stop in open loop. */
  public default void stop() {}
}
