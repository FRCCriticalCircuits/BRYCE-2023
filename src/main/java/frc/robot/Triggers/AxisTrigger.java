package frc.robot.Triggers;

public class AxisTrigger {
    private double axis;
    private double threshold;

    public AxisTrigger(double axis, double threshold) {
        this.axis = axis;
        this.threshold = threshold;
    }

    public AxisTrigger(double axis) {
        this.axis = axis;
        this.threshold = 0.1;
    }

    public boolean trigger() {
        if(axis > threshold){
            return true;
        }
        else {
            return false;
        }
    }
}
