package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private ShooterIO io;
  private ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private static Shooter instance;

  public static Shooter getInstance() {
    return instance;
  }

  public static Shooter initialize(ShooterIO io) {
    if (instance == null) {
      instance = new Shooter(io);
    }
    return instance;
  }

  private Shooter(ShooterIO shooterIO) {
    io = shooterIO;
    io.updateInputs(inputs);
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public void changeVoltage(double dv) {
    setLeftRightVoltage(
      inputs.leftVoltage >= 1 + dv ? inputs.leftVoltage + dv : 0, 
      inputs.rightVoltage >= 1 + dv ? inputs.rightVoltage + dv : 0
    );
  }
 
  public void setLeftRightVoltage(double leftVoltage, double rightVoltage) {
    io.setRightVoltage(rightVoltage);
    io.setLeftVoltage(leftVoltage);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }
}
