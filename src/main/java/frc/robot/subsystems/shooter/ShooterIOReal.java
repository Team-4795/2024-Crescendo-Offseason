package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

public class ShooterIOReal implements ShooterIO {
  private CANSparkMax leftShooterMotor =
      new CANSparkMax(3, MotorType.kBrushless); // Can IDs not accurate
  private CANSparkMax rightShooterMotor =
      new CANSparkMax(12, MotorType.kBrushless); // Can IDs not accurate
  private RelativeEncoder leftShooterEncoder = leftShooterMotor.getEncoder();
  private RelativeEncoder rightShooterEncoder = rightShooterMotor.getEncoder();

  public ShooterIOReal() {
    rightShooterMotor.restoreFactoryDefaults();
    leftShooterMotor.restoreFactoryDefaults();
    leftShooterMotor.setInverted(false);

    rightShooterMotor.setSmartCurrentLimit(40);
    leftShooterMotor.setSmartCurrentLimit(40);

    rightShooterMotor.burnFlash();
    leftShooterMotor.burnFlash();
  }

  @Override
  public void spinForwards() {
    rightShooterMotor.set(1.0);
  }

  @Override
  public void setVoltage(double voltage) {
    rightShooterMotor.setVoltage(voltage);
    leftShooterMotor.setVoltage(voltage);
  }

  @Override
  public void setRightVoltage(double voltage) {
    rightShooterMotor.setVoltage(voltage);
  }

  @Override
  public void setLeftVoltage(double voltage) {
    leftShooterMotor.setVoltage(voltage);
  }

  @Override
  public void spinBackwards() {
    rightShooterMotor.set(-1.0);
  }

  public double getLeftvelocity() {
    return leftShooterEncoder.getVelocity();
  }

  public double getRightvelocity() {
    return rightShooterEncoder.getVelocity();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leftMotorVelocity = getLeftvelocity();
    inputs.rightMotorVelocity = getRightvelocity();
  }
}
