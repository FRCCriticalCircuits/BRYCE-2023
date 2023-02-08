package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class PIDContollers {
    private PIDContollers instance;

    public PIDContollers() {
        
    }

    public void getInstance(){
        if(instance == null) {
            instance = new PIDContollers();
        }
    }

    public PIDController CRITICAL_X() {
        PIDController COMMAND_PID = new PIDController(
            Constants.PIDConstants.CRITICAL_X_PID0_P, 
            Constants.PIDConstants.CRITICAL_X_PID0_I,
            Constants.PIDConstants.CRITICAL_X_PID0_D
        );

        return COMMAND_PID;
    }

    public PIDController CRITICAL_Y() {
        PIDController COMMAND_PID = new PIDController(
            Constants.PIDConstants.CRITICAL_Y_PID0_P, 
            Constants.PIDConstants.CRITICAL_Y_PID0_I,
            Constants.PIDConstants.CRITICAL_Y_PID0_D
        );
            
        return COMMAND_PID;
    }

    public PIDController CRITICAL_THETA() {
        PIDController COMMAND_PID = new PIDController(
            Constants.PIDConstants.CRITICAL_THETA_PID0_P, 
            Constants.PIDConstants.CRITICAL_THETA_PID0_I,
            Constants.PIDConstants.CRITICAL_THETA_PID0_D
        );
            
        return COMMAND_PID; 
    }
}
