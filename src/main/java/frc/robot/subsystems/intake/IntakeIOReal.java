package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class IntakeIOReal implements IntakeIO {
  private CANSparkMax IntakeMotor = new CANSparkMax(IntakeConstants.CanID, MotorType.kBrushless);
  private final RelativeEncoder IntakEncoder = IntakeMotor.getEncoder();

  public IntakeIOReal() {
    IntakeMotor.setSmartCurrentLimit(IntakeConstants.currentLimit);
    IntakeMotor.burnFlash();
  }

  @Override
  public void setMotorSpeed(double speed) {
    IntakeMotor.set(speed);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.angularVelocityRPM = IntakEncoder.getVelocity();
    inputs.angularPositionRot = IntakEncoder.getPosition();
    inputs.Voltage = IntakeMotor.getBusVoltage();
    inputs.Amps = IntakeMotor.getOutputCurrent();
    inputs.noteTime = Intake.time1;
  }
}
