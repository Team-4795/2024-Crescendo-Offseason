package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs{
        public double leftMotorVelocity = 0.0;
        public double rightMotorVelocity = 0.0;
    }

    public default void spinForwards() {}

    public default void spinBackwards() {}

    public default void updateInputs(ShooterIOInputs inputs) {}

}
