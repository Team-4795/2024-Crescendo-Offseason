package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class ShooterIOReal implements ShooterIO {
    private CANSparkFlex leftShooterMotor = new CANSparkFlex(0, MotorType.kBrushless); // Can IDs not accurate
    private CANSparkFlex rightShooterMotor = new CANSparkFlex(1, MotorType.kBrushless); // Can IDs not accurate
    private RelativeEncoder leftShooterEncoder = leftShooterMotor.getEncoder();
    private RelativeEncoder rightShooterEncoder = rightShooterMotor.getEncoder();
    

    public ShooterIOReal() {
        rightShooterMotor.follow(leftShooterMotor, true);

        rightShooterMotor.burnFlash();
        leftShooterMotor.burnFlash();
    }

    public void spinForwards() {
        leftShooterMotor.set(1.0);
    }

    public void spinBackwards() {
        leftShooterMotor.set(-1.0);
    }

    public double getLeftvelocity() {
        return leftShooterEncoder.getVelocity();
    }

    public double getRightvelocity() {
        return rightShooterEncoder.getVelocity();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.leftMotorVelocity = getLeftvelocity();
        inputs.rightMotorVelocity = getRightvelocity();
    }
}
