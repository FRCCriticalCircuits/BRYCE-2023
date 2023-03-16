package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
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
    private boolean EncoderReversed;
    private boolean isPositionControl;
    private PIDController controller = new PIDController(0.003455, 0.000009, 0);

    public Module(int forward_ID, int turn_ID, int cancoder_ID, double angleOffset, boolean EncoderReversed, boolean isPositionControl) {
        this.forward_ID = forward_ID;
        this.turn_ID = turn_ID;
        this.cancoder_ID = cancoder_ID;
        this.angleOffset = angleOffset;
        this.EncoderReversed = EncoderReversed;
        this.isPositionControl = isPositionControl;

        setup();
    }

    public void setup() {
        forward = new CANSparkMax(forward_ID, MotorType.kBrushless);
        turn = new CANSparkMax(turn_ID, MotorType.kBrushless);
        canCoder = new WPI_CANCoder(cancoder_ID);

        forwardPID = forward.getPIDController();
        turnPID = turn.getPIDController();

        forwardPID.setSmartMotionMaxVelocity(Constants.PhysicalConstants.MAX_VELOCITY_RPM, 0);

        controller.enableContinuousInput(-180, 180);

        forward.setSmartCurrentLimit(30);
        turn.setSmartCurrentLimit(30);

        forwardEncoder = forward.getEncoder();
        turnEncoder = turn.getEncoder();

        canCoder.configSensorDirection(EncoderReversed);
        canCoder.configMagnetOffset(angleOffset);
        canCoder.setPosition(getAbsoluteAngle());

        turnEncoder.setPositionConversionFactor(((1 / Constants.PhysicalConstants.ROTATION_GEAR_RATIO) * 360) % 360);
        forwardEncoder.setPositionConversionFactor((1 / Constants.PhysicalConstants.DRIVE_GEAR_RATIO) * Math.PI * 4);
        forwardEncoder.setVelocityConversionFactor((1 / Constants.PhysicalConstants.DRIVE_GEAR_RATIO) * Math.PI * 4);
        canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        forwardPID.setFeedbackDevice(forwardEncoder);
        turnPID.setFeedbackDevice(turnEncoder);
        //controller.setIntegratorRange(-3, 3);

        forwardEncoder.setPosition(0.0);
        turnEncoder.setPosition(canCoder.getAbsolutePosition());

        setNormal();

        forward.burnFlash();
        turn.burnFlash();
    }

    public void setForwardPID(double p, double i, double d, double f, int slotID) {
        forwardPID.setP(p, slotID);
        forwardPID.setI(i, slotID);
        forwardPID.setD(d, slotID);
        forwardPID.setFF(f, slotID);

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

    public void reset(){
        forwardEncoder.setPosition(0);
    }

    public void checkEncoder() {
        if(Math.abs(getAbsoluteAngle()) - (Math.abs(getAngleCancoder()) % 360) > 1){
            turnEncoder.setPosition(getAbsoluteAngle());
        }
    }

    public double closestAngle(double angle) {
        double dir = (angle % 360) - (getAngleCancoder() % 360);

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
            if(isPositionControl){
                turnPID.setReference(getAngleCancoder() + setpointAngle, ControlType.kPosition);
            }else{
                turnPID.setReference(controller.calculate(getAngleCancoder(), getAngleCancoder() + setpointAngle), ControlType.kDutyCycle);
            }
        }else{
            setGain(-1);
            if(isPositionControl){
                turnPID.setReference(getAngleCancoder() + setpointAngleInvert, ControlType.kPosition);
            }else{
                turnPID.setReference(controller.calculate(getAngleCancoder(), getAngleCancoder() + setpointAngleInvert), ControlType.kDutyCycle);
            }
        }
    }

    public void setSpeed(double speed) {
        forward.set(gain * speed);
    }

    public void setState(SwerveModuleState state, boolean isOpenLoop){
        SwerveModuleState.optimize(state, getRotation());
        double velocity = state.speedMetersPerSecond;
        double angle = state.angle.getDegrees();

        setAngle(angle + 180);
        
        if(isOpenLoop){
            setSpeed(velocity / 12);
        }else{
            forwardPID.setReference(Units.metersToInches(velocity) * gain, ControlType.kVelocity);
        }
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

    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(getAngleCancoder());
    }

    public double getDistance() {
        return Units.inchesToMeters(forwardEncoder.getPosition());
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
        //checkEncoder();
    }
}