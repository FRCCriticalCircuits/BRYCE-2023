package frc.robot.Triggers;

import edu.wpi.first.wpilibj.Joystick;

public class AxisTrigger {
    private double axis;

    public AxisTrigger(double axis) {
        this.axis = axis;
    }

    public boolean trigger() {
        if(axis > 0.1){
            return true;
        }
        else {
            return false;
        }
    }
}
