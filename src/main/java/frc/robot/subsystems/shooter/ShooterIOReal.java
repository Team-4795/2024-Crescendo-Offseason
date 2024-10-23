package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class ShooterIOReal implements ShooterIO {
  private CANSparkFlex leftShooterMotor =
      new CANSparkFlex(3, MotorType.kBrushless); // Can IDs not accurate
  private CANSparkFlex rightShooterMotor =
      new CANSparkFlex(12, MotorType.kBrushless); // Can IDs not accurate
  private RelativeEncoder leftShooterEncoder = leftShooterMotor.getEncoder();
  private RelativeEncoder rightShooterEncoder = rightShooterMotor.getEncoder();

  public ShooterIOReal() {
    rightShooterMotor.restoreFactoryDefaults();
    leftShooterMotor.restoreFactoryDefaults();
    leftShooterMotor.follow(rightShooterMotor, true);

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
