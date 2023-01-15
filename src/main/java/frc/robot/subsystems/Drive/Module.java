package frc.robot.subsystems.Drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class Module {
    private CANSparkMax forward;
    private CANSparkMax turn;
    private SparkMaxPIDController forwardPID, turnPID;
    private RelativeEncoder forwardEncoder, turnEncoder;
    private int forward_ID, turn_ID;

    public Module(int forward_ID, int turn_ID) {
        this.forward_ID = forward_ID;
        this.turn_ID = turn_ID;

        setup();
    }

    public void setup() {
        forward = new CANSparkMax(forward_ID, MotorType.kBrushless);
        turn = new CANSparkMax(turn_ID, MotorType.kBrushless);

        forwardPID = forward.getPIDController();
        turnPID = turn.getPIDController();

        forwardEncoder = forward.getEncoder();
        turnEncoder = turn.getEncoder();

        turnEncoder.setPositionConversionFactor(((1 / Constants.PhysicalConstants.ROTATION_GEAR_RATIO) * 360) % 360);
        forwardEncoder.setPositionConversionFactor((1 / Constants.PhysicalConstants.DRIVE_GEAR_RATIO) * Math.PI * 4);

        forwardPID.setFeedbackDevice(forwardEncoder);
        turnPID.setFeedbackDevice(turnEncoder);

        forwardEncoder.setPosition(0.0);
        turnEncoder.setPosition(0.0);
    }

    public void setForwardPID(double p, double i, double d, double f, int slotID) {
        forwardPID.setP(p, slotID);
        forwardPID.setI(i, slotID);
        forwardPID.setD(d, slotID);
        forwardPID.setFF(f, slotID);
        forwardPID.setOutputRange(-1, 1);
    }    
    
    public void setTurnPID(double p, double i, double d, double f, int slotID) {
        turnPID.setP(p, slotID);
        turnPID.setI(i, slotID);
        turnPID.setD(d, slotID);
        turnPID.setFF(f, slotID);
        turnPID.setOutputRange(-90, 90);
    }

    public void setNormal(){
        forward.setIdleMode(IdleMode.kCoast);
        turn.setIdleMode(IdleMode.kBrake);
    }

    public void setBrakeMode(Boolean mode){
        if(mode){
            forward.setIdleMode(IdleMode.kBrake);
            turn.setIdleMode(IdleMode.kBrake);
        }else{
            forward.setIdleMode(IdleMode.kCoast);
            turn.setIdleMode(IdleMode.kCoast);        
        }
    }

    public void resetEncoderForward() {
        forwardEncoder.setPosition(0);
    }

    public void resetEncoderTurn() {
        turnEncoder.setPosition(0);
    }

    public double closestAngle(double angle) {
        double dir = (angle % 360) - (getAngle() % 360);

        if(Math.abs(dir) > 180){
            dir = -(Math.signum(dir) * 360) + dir;
        }
        
        return dir;
    }

    public void setAngle(double angle) {
        double setpointAngle = closestAngle(angle);
        double setpointAngleInvert = closestAngle(angle + 180);
        turnPID.setReference(getAngle() + closestAngle(angle), ControlType.kPosition);

        if(Math.abs(setpointAngle) <= Math.abs(setpointAngleInvert)){
            
            setInverted(false);
            turnPID.setReference(getAngle() + setpointAngle, ControlType.kPosition);
        }else{

            setInverted(true);
            turnPID.setReference(getAngle() + setpointAngleInvert, ControlType.kPosition);
        }
    }

    public void setSpeed(double speed) {
        forward.set(speed);
    }

    public double getSpeed(){
        return Units.inchesToMeters(
            forwardEncoder.getVelocity() / 60
        );
    }

    public void moveToInches(double distance){
        forwardPID.setReference(distance, ControlType.kPosition);
    }

    public double getAngle() {
        return (turnEncoder.getPosition());
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getAngle());
    }

    public double getDistance() {
        return (forwardEncoder.getPosition());
    }

    public void setInverted(boolean invert) {
        if(invert){
            forward.setInverted(true);
        }else{
            forward.setInverted(false);
        }
    }
}