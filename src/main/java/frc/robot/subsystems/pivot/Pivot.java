package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.pivot.PivotIO;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {
    
    private static Pivot pivotInstance = null;
    public final PivotIO io;
    private final SimpleMotorFeedforward ffModel;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    private Pivot(PivotIO io) {
        
        this.io = io;

        switch (Constants.currentMode) {
            case REAL:
            case REPLAY:
              ffModel = new SimpleMotorFeedforward(0.0, 0.0);
              io.configurePID(1.0, 0.0, 0.0);
              break;
            case SIM:
              ffModel = new SimpleMotorFeedforward(0.0, 0.0);
              io.configurePID(1.0, 0.0, 0.0);
              break;
            default:
              ffModel = new SimpleMotorFeedforward(0.0, 0.0);
              break;
        }


    }

    public static Pivot getInstance() {
        return pivotInstance;
    }

    public static Pivot initialize(PivotIO io) {
        if (Pivot.pivotInstance == null) {
            pivotInstance = new Pivot(io);
        }
        return pivotInstance;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);
    }

    public void runVoltage(double volts) {
        io.setVoltage(volts);
    }

    public void stop() {
        io.stop();
    }


}