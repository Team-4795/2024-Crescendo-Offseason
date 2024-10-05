package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private final DCMotorSim motor = new DCMotorSim(DCMotor.getNeoVortex(1), 1, 1);

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    motor.update(0.01);

    inputs.Amps = motor.getCurrentDrawAmps();
  }

  @Override
  public void setMotorSpeed(double speed) {
    motor.setInputVoltage(speed);
  }
}
