package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner14;

import org.opencv.core.Mat;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.GoalType.goalType;

public class LimelightSubsystem extends SubsystemBase {
    private NetworkTable table;
    private double[] defaultArray = {0, 0, 0, 0, 0, 0};
    private double Offset;
    private goalType commandGoad;
    private goalType currentGoal;

    public LimelightSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        currentGoal = goalType.NONE;
    }

    public void setPipeline(int pipeline){
        table.getEntry("pipeline").setNumber(pipeline);
    }

    public void setLEDMode(int ledMode){ // 0 for default. 1 for on, 2 for flash and 3 for off
        table.getEntry("ledmode").setNumber(ledMode);
    }

    public void setGoalType(goalType goal){
        currentGoal = goal;
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

    public double getId() {
        return table.getEntry("tid").getDouble(0);
    }

    public double[] getbotpose_targetspace(){
        return table.getEntry("botpose_targetspace").getDoubleArray(defaultArray);
    }

    public double getXPoseOffset() {
        return Math.abs(getbotpose_targetspace()[0]);
    }

    public double getRawTargetDistance() {
        return getbotpose_targetspace()[2];
    }

    public goalType getCurrentGoal() {
        return currentGoal;
    }

    public double getGoalDistance() {
        double rawdistance, goalDistance;

        if(currentGoal == goalType.MID){
            Offset = 0.216027;
        }else if(currentGoal == goalType.HIGH){
            Offset = 0.654685;
        }else{
            Offset = 0;
        }

        rawdistance = Math.sqrt((getRawTargetDistance() * getRawTargetDistance()) - (getXPoseOffset() * getXPoseOffset()));
        goalDistance = Math.sqrt(((rawdistance + Offset) * (rawdistance + Offset)) + (getXPoseOffset() * getXPoseOffset()));

        return goalDistance;
    }

    public double getBestGoalOffset(){
        double angularOffset = 0;

        if(Math.abs(Offset) > 0) {
            angularOffset = Math.acos(((Offset * Offset) + Math.abs(getRawTargetDistance() * getRawTargetDistance()) - Math.abs(getGoalDistance() * getGoalDistance())) / 2 * Math.signum(getRawTargetDistance()) * getRawTargetDistance() * Offset);
        }

        return Math.signum(getXOffset()) * angularOffset;
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("TARGET ID", getId());
        SmartDashboard.putNumber("TARGET DISTANCE", getRawTargetDistance());
        SmartDashboard.putBoolean("IS TARGET", isTarget());
        SmartDashboard.putString("GoalType", currentGoal.toString());
        SmartDashboard.putNumber("GET BEST OFFSET", getXOffset() + getBestGoalOffset());
    }

}