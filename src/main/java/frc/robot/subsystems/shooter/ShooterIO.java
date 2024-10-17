package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double topMotorVelocity = 0.0;
    public double bottomMotorVelocity = 0.0;
    public double topAppliedVolts = 0.0;
    public double bottomAppliedVolts = 0.0;
    public double topOutputCurrent = 0.0;
    public double bottomOutputCurrent = 0.0;
  }

  public default void setVoltage(double voltage) {}

  public default void setVelocity(double velocity) {}

  public default void updateInputs(ShooterIOInputs inputs) {}
}
