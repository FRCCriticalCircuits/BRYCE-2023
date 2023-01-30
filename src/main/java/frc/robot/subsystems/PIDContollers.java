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

    public PIDController COMMAND_X_PID() {
        PIDController COMMAND_PID = new PIDController(
            1, 
            0, 
            0
        );

        return COMMAND_PID;
    }

    public PIDController COMMAND_Y_PID() {
        PIDController COMMAND_PID = new PIDController(
            1, 
            0,
            0
        );
            
        return COMMAND_PID;
    }

    public PIDController THETA_PID() {
        PIDController COMMAND_PID = new PIDController(
            1, 
            0,
            0
        );
            
        return COMMAND_PID; 
    }
}
