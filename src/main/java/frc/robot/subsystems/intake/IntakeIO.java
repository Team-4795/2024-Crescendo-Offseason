package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double angularVelocityRPM = 0.0;
    public double angularPositionRot = 0.0;
    public double Voltage = 0.0; // ..
    public double Amps = 0.0;
    public double noteTime = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setMotorSpeed(double speed) {}
}
