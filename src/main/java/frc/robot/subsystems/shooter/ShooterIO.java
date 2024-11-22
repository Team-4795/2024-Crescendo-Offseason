package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double leftMotorVelocity = 0.0;
    public double rightMotorVelocity = 0.0;
    public double leftVoltage = 0.0;
    public double rightVoltage = 0.0;
  }

  public default void spinForwards() {}

  public default void spinBackwards() {}

  public default void setVoltage(double voltage) {}

  public default void setRightVoltage(double voltage) {}

  public default void setLeftVoltage(double voltage) {}

  public default void updateInputs(ShooterIOInputs inputs) {}
}
