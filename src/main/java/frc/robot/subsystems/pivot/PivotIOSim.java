package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotIOSim implements PivotIO {
  private SingleJointedArmSim sim =
  new SingleJointedArmSim(DCMotor.getNEO(1), 1.5, 0.004, 0.2, 0, Math.PI, false, 0);
  private PIDController pid = new PIDController(0.2, 0.0, 0.0);

  private boolean closedLoop = true;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    sim.update(0.02);

    appliedVolts = MathUtil.clamp(pid.calculate(sim.getAngleRads()) + ffVolts, -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);

    inputs.positionRad = sim.getAngleRads();
    inputs.appliedVolts = appliedVolts;
    inputs.velocityRadPerSec = sim.getVelocityRadPerSec();
  }

  @Override
  public void setAngle(double targetAngle) {
    pid.setSetpoint(targetAngle);
  }
  
  @Override
  public void setVoltage(double volts) {
    sim.setInputVoltage(volts);
  }
}
