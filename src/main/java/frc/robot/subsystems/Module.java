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
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private boolean driveMotorReversed;
    private boolean isPositionControl;
    private PIDController controller = new PIDController(0.003455, 0.000009, 0);
    //private PIDController driveController = new PIDController(0, 0, 0);

    /**
     * SWERVE MODULE OF A SWERVE DRIVE
     * 
     * @param forward_ID ID of the drive motor
     * @param turn_ID ID of the turn motor
     * @param cancoder_ID ID of the module CANCoder
     * @param angleOffset Offset of the module CANCoder
     * @param EncoderReversed If the CANcoder is reversed or not
     * @param isPositionControl If the turn PIDs are position control or DutyCycle
     */
    public Module(int forward_ID, int turn_ID, int cancoder_ID, double angleOffset, boolean driveMotorReversed, boolean EncoderReversed, boolean isPositionControl) {
        this.forward_ID = forward_ID;
        this.turn_ID = turn_ID;
        this.cancoder_ID = cancoder_ID;
        this.angleOffset = angleOffset;
        this.EncoderReversed = EncoderReversed;
        this.isPositionControl = isPositionControl;
        this.driveMotorReversed = driveMotorReversed;

        setup();
    }

    public void setup() {
        forward = new CANSparkMax(forward_ID, MotorType.kBrushless);
        turn = new CANSparkMax(turn_ID, MotorType.kBrushless);
        canCoder = new WPI_CANCoder(cancoder_ID);

        forwardPID = forward.getPIDController();
        turnPID = turn.getPIDController();

        forwardPID.setSmartMotionMaxVelocity(Constants.PhysicalConstants.MAX_VELOCITY_RPM, 0);

        //controller.enableContinuousInput(-180, 180);

        forward.setSmartCurrentLimit(40);
        turn.setSmartCurrentLimit(40);

        forwardEncoder = forward.getEncoder();
        turnEncoder = turn.getEncoder();

        forward.setInverted(driveMotorReversed);
        turn.setInverted(true);

        canCoder.configSensorDirection(EncoderReversed);
        canCoder.configMagnetOffset(angleOffset);
        canCoder.setPosition(getAbsoluteAngle());

        turnEncoder.setPositionConversionFactor(((1 / Constants.PhysicalConstants.ROTATION_GEAR_RATIO) * 360));
        forwardEncoder.setPositionConversionFactor(Units.inchesToMeters((1 / Constants.PhysicalConstants.DRIVE_GEAR_RATIO * Math.PI * 4)));
        forwardEncoder.setVelocityConversionFactor(Units.inchesToMeters(((1 / Constants.PhysicalConstants.DRIVE_GEAR_RATIO * Math.PI * 4))) / 60);
        canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

        forwardPID.setFeedbackDevice(forwardEncoder);
        turnPID.setFeedbackDevice(turnEncoder);
        //controller.setIntegratorRange(-3, 3);

        forwardEncoder.setPosition(0.0);
        turnEncoder.setPosition(canCoder.getAbsolutePosition());

        setBrakeMode(true);

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
        //turnPID.setOutputRange(-180, 180);

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

    /**
     * Increases the maximum rate a motor can output power
     * 
     * @param rate The ramp rate
     * @param isOpenLoop true if ramp up in open loop and false to ramp up in closed loop
     */
    public void setDriveMotorRampRate(double rate, boolean isOpenLoop) {
        if(isOpenLoop) {
            forward.setOpenLoopRampRate(rate);
        }else{
            forward.setClosedLoopRampRate(rate);
        }
    }

    /**
     * 
     * @return Stops the drive motor
     */
    public void stopMotors() {
        forward.setVoltage(0);
    }

    public void reset(){
        forwardEncoder.setPosition(0);
        canCoder.setPosition(getAbsoluteAngle());
    }

    public void checkEncoder() {
        if(Math.abs(getAbsoluteAngle()) - (Math.abs(getAngleCancoder()) % 360) > 1){
            canCoder.setPosition(getAbsoluteAngle());
        }
    }

    public double closestAngle(double angle) {
        double dir = (angle % 360) - (getAngleCancoder() % 360);

        if(Math.abs(dir) > 180){
            dir = -(Math.signum(dir) * 360) + dir;
        }
        
        return dir;
    }

    /**
     * 
     * @param angle Desired angle of the modules wheel
     * 
     * @return Changes the angle of the module's wheel to desired angle
     */
    public void setAngle(double angle) {
        double setpointAngle = closestAngle(angle);
        double setpointAngleInvert = closestAngle(angle + 180);

        //double setpointAngle = angle;
        //double setpointAngleInvert = angle + 180;

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

    /**
     * 
     * @param speed Desired speed of the drive motors
     */
    public void setSpeed(double speed) {
        forward.set(gain * speed);
        //forward.set(speed);
    }

    /**
     * 
     * @param state Desired swerve module state
     * @param isOpenLoop Is method being used in open loop
     * 
     * @return Transforms module to desired state
     */
    public void setState(SwerveModuleState state, boolean isOpenLoop){
        SwerveModuleState desiredState = SwerveModuleState.optimize(state, getRotation());
        double velocity = desiredState.speedMetersPerSecond;
        double moduleangle = desiredState.angle.getDegrees();
        
        if(isOpenLoop){
            setAngle(moduleangle);
            setSpeed(velocity / 5);
            //turnPID.setReference(controller.calculate(getAngleCancoder(), getAngleCancoder() + closestAngle(moduleangle)), ControlType.kDutyCycle);
            //setAngle(state.angle.getDegrees());
            //forwardPID.setReference(velocity * gain, ControlType.kVelocity);
        }else{
            forwardPID.setReference(velocity, ControlType.kVelocity);
            //setSpeed(velocity / 12);
            turnPID.setReference(controller.calculate(getAngleCancoder(), getAngleCancoder() + closestAngle(moduleangle)), ControlType.kDutyCycle);
        }

        SmartDashboard.getNumber("TARGET ANGLE " + cancoder_ID / 3, moduleangle);
        SmartDashboard.getNumber("ANGLE " + cancoder_ID / 3, moduleangle);
    }

    public void setGain(double gain){
        this.gain = gain;
    }

    /**
     * 
     * @return Speed of the drive motor in meters/sec
     */
    public double getSpeed(){
        return forwardEncoder.getVelocity();
    }

    /**
     * 
     * @return Angle of the module wheel from hall effect encoders
     */
    public double getAngle() {
        return turnEncoder.getPosition();
    }

    /**
     * 
     * @return Angle of the module wheel from the CANcoder
     */
    public double getAngleCancoder() {
        return canCoder.getPosition();
    }

    /**
     * 
     * @return Angle of the Absolute encoders
     */
    public double getAbsoluteAngle() {
        return canCoder.getAbsolutePosition();
    }

    /**
     * 
     * @return Angle of swerve wheel in Rotation2d
     */
    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(getAngleCancoder());
    }

    /**
     * 
     * @return Distance traveled by drive motors in meters
     */
    public double getDistance() {
        return forwardEncoder.getPosition();
    }

    /* DEPRECIATED FOR REMOVAL
    public void moveToInches(double distance){
        forwardPID.setReference(distance, ControlType.kPosition);
    }
    */

    /**
     * 
     * @return The state of the module
     */
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(
           getSpeed(), getRotation()
        );
    }

    /**
     * 
     * @return The position of the module
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
            getDistance(), 
            getRotation()
        );
    }

    @Override
    public void periodic(){
        //checkEncoder();
        //SmartDashboard.putString("MODULE[" + cancoder_ID / 3 + "] :", getModuleState().toString());
    }
}