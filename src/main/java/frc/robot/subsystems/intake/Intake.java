package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private static Intake instance;
    public double intakespd;

    private Intake(IntakeIO IO) {
        io = IO;
    }

    public static Intake getInstance() {
        return instance;
    }

    public static Intake initialize(IntakeIO io) {
        if (instance == null) {
            instance = new Intake(io);
        }
        return instance;
    }

    public void setIntakeSpeed(double speed){
        intakespd = speed;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        io.setMotorSpeed(intakespd);
    }
}
