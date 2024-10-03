package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class IntakeIOReal implements IntakeIO {
    private CANSparkMax IntakeMotor = new CANSparkMax(IntakeConstants.CanID, MotorType.kBrushless);

    public IntakeIOReal() {
        IntakeMotor.setSmartCurrentLimit(IntakeConstants.currentLimit);
        IntakeMotor.burnFlash();
    }

    @Override
    public void setMotorSpeed(double speed) {
        IntakeMotor.set(speed);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.Voltage = IntakeMotor.getBusVoltage();
        inputs.Amps = IntakeMotor.getOutputCurrent();
    }

}
