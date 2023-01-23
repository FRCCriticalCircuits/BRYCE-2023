package frc.robot.subsystems;

import java.security.CodeSource;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LimelightSubsystem extends SubsystemBase {
    private NetworkTable table;

    private enum TargetType{
        GOAL, 
        PICKUP,
        NONE
    };

    private TargetType Target;

    public LimelightSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public void setPipeline(int pipeline){
        table.getEntry("pipeline").setNumber(pipeline);
    }

    public void setLEDMode(int ledMode){ // 0 for default. 1 for on, 2 for flash and 3 for off
        table.getEntry("ledmode").setNumber(ledMode);
    }

    public boolean isTarget(){
        return table.getEntry("tv").getBoolean(false);
    }

    public double getXOffset() {
        return table.getEntry("tx").getDouble(0);
    }

    public double getYOffset() {
        return table.getEntry("ty").getDouble(0);
    }

    public double getArea() {
        return table.getEntry("ta").getDouble(0);
    }

    public int getId() {
        return (int) table.getEntry("tid").getInteger(0);
    }

    public TargetType getTarget(){
        return Target;
    }

    public double getDistanceToTarget(int id){
        double distance;
        double height = 0;

        switch(id){
            case 1:
            case 2:
            case 3:
            case 6:
            case 7:
            case 8:
                height = Constants.PhysicalConstants.LL_CUBE_GOAL_HEIGHT;
                Target = TargetType.GOAL;
            case 4:
            case 5:
                height = Constants.PhysicalConstants.LL_PICKUP_GOAL_HEIGHT;
                Target = TargetType.PICKUP;
            default:
                Target = TargetType.NONE;
        }

        distance = (height + Constants.PhysicalConstants.LL_HEIGHT) / Math.tan(getYOffset());

        return distance;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("TARGET DISTANCE", getDistanceToTarget(getId()));
        SmartDashboard.putBoolean("IS TARGET", isTarget());
        SmartDashboard.putNumber("PIPELINE", getArea());
    }

}