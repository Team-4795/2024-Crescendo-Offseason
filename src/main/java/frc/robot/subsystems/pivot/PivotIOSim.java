package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.flywheel.FlywheelIO.FlywheelIOInputs;

public class PivotIOSim implements PivotIO {
    private SingleJointedArmSim sim = new SingleJointedArmSim(DCMotor.getNEO(1), 1.5, 
    0.004, 0, 0, 0, 
    false, 0);
    private PIDController pid = new PIDController(1.0, 0.0, 0.0);

    private boolean closedLoop = false;
    private double ffVolts = 0.0;
    private double appliedVolts = 0.0;
    
      @Override
    public void updateInputs(PivotIOInputs inputs) {
        if (closedLoop) {
        appliedVolts =
            MathUtil.clamp(pid.calculate(sim.getVelocityRadPerSec()) + ffVolts, -12.0, 12.0);
            sim.setInputVoltage(appliedVolts);
        }
    }

    @Override
    public void rotatePivot(double speed) {
        pivotAppliedVolts = MathUtil.clamp(12 * speed, -12, 12);
        sim.setInputVoltage(appliedVolts);
    }

    @Override
    public void setVoltage(double volts) {
        sim.setInputVoltage(volts);
    }


}
