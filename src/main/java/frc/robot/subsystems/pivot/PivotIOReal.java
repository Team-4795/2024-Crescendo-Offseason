package frc.robot.subsystems.pivot;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class PivotIOReal implements PivotIO {
  private CANSparkFlex pivotLeft = new CANSparkFlex(10, MotorType.kBrushless);
  private CANSparkFlex pivotRight = new CANSparkFlex(11, MotorType.kBrushless);
  private RelativeEncoder pivotEncoderRight = pivotRight.getEncoder();
  private PIDController pid = new PIDController(1.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  public PivotIOReal() {
    pivotLeft.restoreFactoryDefaults();
    pivotRight.restoreFactoryDefaults();

    pivotLeft.follow(pivotRight);
    pivotLeft.setInverted(true);

    pivotLeft.setCANTimeout(250);
    pivotRight.setCANTimeout(250);
    pivotLeft.enableVoltageCompensation(12);
    pivotRight.enableVoltageCompensation(12);
    pivotLeft.setSmartCurrentLimit(80);
    pivotRight.setSmartCurrentLimit(80);
    pivotLeft.setIdleMode(IdleMode.kBrake);
    pivotRight.setIdleMode(IdleMode.kBrake);

    pivotLeft.burnFlash();
    pivotRight.burnFlash();
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.positionRad = pivotEncoderRight.getPosition() * 2 * Math.PI;
    inputs.velocityRevolutionsPerSec = pivotEncoderRight.getVelocity();
    inputs.appliedVolts = appliedVolts;

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
    pivotRight.setVoltage(appliedVolts);
  }
}
