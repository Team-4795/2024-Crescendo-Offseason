package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class ShooterIOReal implements ShooterIO {
  private CANSparkFlex topShooterMotor =
      new CANSparkFlex(0, MotorType.kBrushless); // Can IDs not accurate
  private CANSparkFlex bottomShooterMotor =
      new CANSparkFlex(1, MotorType.kBrushless); // Can IDsp not accurate

  private RelativeEncoder topShooterEncoder = topShooterMotor.getEncoder();
  private RelativeEncoder bottomShooterEncoder = bottomShooterMotor.getEncoder();
  private SimpleMotorFeedforward ffmodel = new SimpleMotorFeedforward(0, 0);

  public ShooterIOReal() {
    bottomShooterMotor.restoreFactoryDefaults();
    topShooterMotor.restoreFactoryDefaults();

    bottomShooterMotor.follow(topShooterMotor, true);

    topShooterMotor.setSmartCurrentLimit(40);
    bottomShooterMotor.setSmartCurrentLimit(40);

    bottomShooterMotor.setIdleMode(IdleMode.kCoast);
    topShooterMotor.setIdleMode(IdleMode.kCoast);

    bottomShooterMotor.burnFlash();
    topShooterMotor.burnFlash();
  }

  @Override
  public void setVoltage(double voltage) {
    topShooterMotor.setVoltage(voltage);
  }

  @Override
  public void setVelocity(double velocity) {
    topShooterMotor.setVoltage(ffmodel.calculate(velocity));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.topMotorVelocity = topShooterEncoder.getVelocity();
    inputs.bottomMotorVelocity = bottomShooterEncoder.getVelocity();
    inputs.topAppliedVolts = topShooterMotor.getBusVoltage();
    inputs.bottomAppliedVolts = bottomShooterMotor.getBusVoltage();
    inputs.topOutputCurrent = topShooterMotor.getOutputCurrent();
    inputs.bottomOutputCurrent = bottomShooterMotor.getOutputCurrent();
  }
}
