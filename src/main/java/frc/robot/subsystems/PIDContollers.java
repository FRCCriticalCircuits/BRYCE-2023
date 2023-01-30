package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;

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
            1, 
            0, 
            0
        );

        return COMMAND_PID;
    }

    public PIDController CRITICAL_Y() {
        PIDController COMMAND_PID = new PIDController(
            1, 
            0,
            0
        );
            
        return COMMAND_PID;
    }

    public PIDController CRITICAL_THETA() {
        PIDController COMMAND_PID = new PIDController(
            1, 
            0,
            0
        );
            
        return COMMAND_PID; 
    }
}
