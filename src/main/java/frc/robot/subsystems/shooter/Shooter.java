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
}
