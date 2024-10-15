package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private final DCMotorSim motor = new DCMotorSim(DCMotor.getNeoVortex(1), 30, 0.003);

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    motor.update(0.02);

    inputs.angularPositionRot = motor.getAngularPositionRotations();
    inputs.angularVelocityRPM = motor.getAngularVelocityRPM();
    inputs.Amps = motor.getCurrentDrawAmps();
    inputs.noteTime = Intake.time1;
  }

  @Override
  public void setMotorSpeed(double speed) {
    motor.setInputVoltage(speed);
  }
}
