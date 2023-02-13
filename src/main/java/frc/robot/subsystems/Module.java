package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Module extends SubsystemBase{
    private CANSparkMax forward;
    private CANSparkMax turn;
    private WPI_CANCoder canCoder;
    private SparkMaxPIDController forwardPID, turnPID;
    private RelativeEncoder forwardEncoder, turnEncoder;
    private int forward_ID, turn_ID, cancoder_ID;
    private double gain, angleOffset;

    

    public Module(int forward_ID, int turn_ID, int cancoder_ID, double angleOffset) {
        this.forward_ID = forward_ID;
        this.turn_ID = turn_ID;
        this.cancoder_ID = cancoder_ID;
        this.angleOffset = angleOffset;

        setup();
    }

    public void setup() {
        forward = new CANSparkMax(forward_ID, MotorType.kBrushless);
        turn = new CANSparkMax(turn_ID, MotorType.kBrushless);
        canCoder = new WPI_CANCoder(cancoder_ID);

        forwardPID = forward.getPIDController();
        turnPID = turn.getPIDController();

        forwardPID.setSmartMotionMaxVelocity(Constants.PhysicalConstants.MAX_VELOCITY_RPM, 0);

        forwardEncoder = forward.getEncoder();
        turnEncoder = turn.getEncoder();

        turnEncoder.setPositionConversionFactor(((1 / Constants.PhysicalConstants.ROTATION_GEAR_RATIO) * 360) % 360);
        forwardEncoder.setPositionConversionFactor((1 / Constants.PhysicalConstants.DRIVE_GEAR_RATIO) * Math.PI * 4);
        canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        forwardPID.setFeedbackDevice(forwardEncoder);
        turnPID.setFeedbackDevice(turnEncoder);

        forwardEncoder.setPosition(0.0);
        turnEncoder.setPosition(0.0);
        canCoder.setPosition(0);

        forward.burnFlash();
        turn.burnFlash();
    }

    public void setForwardPID(double p, double i, double d, double f, int slotID) {
        forwardPID.setP(p, slotID);
        forwardPID.setI(i, slotID);
        forwardPID.setD(d, slotID);
        forwardPID.setFF(f, slotID);
        forwardPID.setOutputRange(-1, 1);

        forward.burnFlash();
    }    
    
    public void setTurnPID(double p, double i, double d, double f, int slotID) {
        turnPID.setP(p, slotID);
        turnPID.setI(i, slotID);
        turnPID.setD(d, slotID);
        turnPID.setFF(f, slotID);
        turnPID.setIZone(1);
        turnPID.setOutputRange(-180, 180);

        turn.burnFlash();
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

    public void stopMotors() {
        forward.setVoltage(0);
        turn.setVoltage(0);
    }

    public void resetEncoderForward() {
        forwardEncoder.setPosition(0);
    }

    public void resetEncoderTurn() {
        turnEncoder.setPosition(0);
    }

    public void resetEncoders(){
        forwardEncoder.setPosition(0);
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

        if(Math.abs(setpointAngle) <= Math.abs(setpointAngleInvert)){
            
            setGain(1);
            turnPID.setReference(getAngle() + setpointAngle, ControlType.kPosition);
        }else{

            setGain(-1);
            turnPID.setReference(getAngle() + setpointAngleInvert, ControlType.kPosition);
        }
    }

    public void setSpeed(double speed) {
        forward.set(gain * speed);
    }

    public void setState(SwerveModuleState state){
        double velocity = state.speedMetersPerSecond * 60;
        double angle = state.angle.getDegrees();

        setAngle(angle);
        forwardPID.setReference(velocity, ControlType.kVelocity);
    }

    public void setGain(double gain){
        this.gain = gain;
    }

    public double getSpeed(){
        return Units.inchesToMeters(
            forwardEncoder.getVelocity() / 60
        );
    }

    public double getAngle() {
        return turnEncoder.getPosition();
    }

    public double getAngleCancoder() {
        return canCoder.getPosition();
    }

    public double getAbsoluteAngle() {
        return canCoder.getAbsolutePosition();
    }

    //public double getAbsoluteAngle() {
    
    //}

    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(getAngle());
    }

    public double getDistance() {
        return (forwardEncoder.getPosition());
    }

    /* DEPRECIATED FOR REMOVAL
    public void moveToInches(double distance){
        forwardPID.setReference(distance, ControlType.kPosition);
    }
    */

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
           getSpeed(), getRotation()
        );
    }

    @Override
    public void periodic(){
        
    }

}