package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;

public class GyroIONavX implements GyroIO {
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  public GyroIONavX() {
    reset();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = gyro.isConnected();
    inputs.yawPosition = gyro.getRotation2d();
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(gyro.getRate());
  }

  @Override
  public void reset() {
    gyro.reset();
  }

  public void setOffset(double offset) {
    gyro.setAngleAdjustment(offset);
  }
}
