package frc.robot.subsystems.pivot;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class PivotIOReal implements PivotIO {
  private CANSparkMax pivotMotor = new CANSparkMax(11, MotorType.kBrushless);
  private RelativeEncoder pivotEncoderRight = pivotMotor.getEncoder();
  private PIDController pid = new PIDController(1.0, 0.0, 0.0);
  private double gearing = 36;

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  public PivotIOReal() {
    pivotMotor.restoreFactoryDefaults();

    pivotMotor.setCANTimeout(250);
    pivotMotor.enableVoltageCompensation(12);
    pivotMotor.setSmartCurrentLimit(30);
    pivotMotor.setIdleMode(IdleMode.kBrake);

    pivotMotor.burnFlash();
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.positionRad = (pivotEncoderRight.getPosition() * 2 * Math.PI) / gearing;
    inputs.velocityRadPerSec = (pivotEncoderRight.getVelocity() * 2 * Math.PI / 60) / gearing;
    inputs.appliedVolts = appliedVolts;
    inputs.goalAngle = pid.getSetpoint();

    if (closedLoop) {
      appliedVolts = MathUtil.clamp(pid.calculate(inputs.positionRad) + ffVolts, -12.0, 12.0);
    }
  }

  @Override
  public void setAngle(double targetAngle) {
    pid.setSetpoint(targetAngle);
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12, 12);
    pivotMotor.setVoltage(appliedVolts);
  }
}
