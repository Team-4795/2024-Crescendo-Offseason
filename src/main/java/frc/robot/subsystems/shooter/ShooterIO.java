package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double leftMotorVelocity = 0.0;
    public double rightMotorVelocity = 0.0;
    public double leftAppliedVolts = 0.0;
    public double rightAppliedVolts = 0.0;
    public double leftOutputCurrent = 0.0;
    public double rightOutputCurrent = 0.0;
  }

  public default void spinForwards() {}

  public default void spinBackwards() {}

  public default void stopMotor() {}

  public default void updateInputs(ShooterIOInputs inputs) {}
}
