// thomas jin fix this
// i just copy pasta from flywheel sim
// type shi pray emoji
package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ShooterIOSim implements ShooterIO {
  private DCMotorSim sim = new DCMotorSim(DCMotor.getNEO(1), 1.5, 0.004);
  private double appliedVolts = 0.0;
  private SimpleMotorFeedforward ffmodel = new SimpleMotorFeedforward(0, 0);

  @Override
  public void setVoltage(double voltage) {
    appliedVolts = voltage;
    sim.setInputVoltage(voltage);
  }

  @Override
  public void setVelocity(double velocity) {
    setVoltage(ffmodel.calculate(velocity));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    sim.update(0.02);

    inputs.topMotorVelocity = sim.getAngularVelocityRPM();
    inputs.bottomMotorVelocity = sim.getAngularVelocityRPM();
    inputs.topAppliedVolts = appliedVolts;
    inputs.bottomAppliedVolts = appliedVolts;
    inputs.topOutputCurrent = sim.getCurrentDrawAmps();
    inputs.bottomOutputCurrent = sim.getCurrentDrawAmps();
  }
}
