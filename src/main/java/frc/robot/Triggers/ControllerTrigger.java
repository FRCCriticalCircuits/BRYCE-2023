package frc.robot.Triggers;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControllerTrigger extends Trigger{
    DoubleSupplier axis;
    double threshold;

    public ControllerTrigger(DoubleSupplier axis) {
        this.axis = axis;
        this.threshold = threshold;
    }

    
    public boolean trigger() {
        if(axis.getAsDouble() > 0.85){
            return true;
        }else{
            return false;
        }
    }
}
