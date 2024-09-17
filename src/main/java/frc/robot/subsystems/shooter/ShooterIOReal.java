package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class ShooterIOReal implements ShooterIO {
  private CANSparkFlex leftShooterMotor =
      new CANSparkFlex(0, MotorType.kBrushless); // Can IDs not accurate
  private CANSparkFlex rightShooterMotor =
      new CANSparkFlex(1, MotorType.kBrushless); // Can IDsp not accurate
  private RelativeEncoder leftShooterEncoder = leftShooterMotor.getEncoder();
  private RelativeEncoder rightShooterEncoder = rightShooterMotor.getEncoder();

  public ShooterIOReal() {
    rightShooterMotor.follow(leftShooterMotor, true);
   
    leftShooterMotor.setSmartCurrentLimit(40);
    rightShooterMotor.setSmartCurrentLimit(40);

    rightShooterMotor.burnFlash();
    leftShooterMotor.burnFlash();
  }

  @Override
  public void spinForwards() {
    leftShooterMotor.set(1.0);
  }

  @Override
  public void spinBackwards() {
    leftShooterMotor.set(-1.0);
  }

  @Override
  public void stopMotor() {
    leftShooterMotor.set(0.0);
  }

  public double getLeftvelocity() {
    return leftShooterEncoder.getVelocity();
  }

  public double getRightvelocity() {
    return rightShooterEncoder.getVelocity();
  }

  public double getLeftVoltage() {
    return leftShooterMotor.getBusVoltage();
  }

  public double getRightVoltage() {
      return rightShooterMotor.getBusVoltage();
  }

  public double getLeftCurrent() {
      return leftShooterMotor.getOutputCurrent();
  }

  public double getRightCurrent() {
      return rightShooterMotor.getOutputCurrent();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leftMotorVelocity = getLeftvelocity();
    inputs.rightMotorVelocity = getRightvelocity();
    inputs.leftAppliedVolts = getLeftVoltage(); 
    inputs.rightAppliedVolts = getRightVoltage();
    inputs.leftOutputCurrent = getLeftCurrent(); 
    inputs.rightOutputCurrent = getRightCurrent();
  }
}

