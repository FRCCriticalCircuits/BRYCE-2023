package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private CANSparkMax flywheel_Top, flywheel_buttom;
    private SparkMaxPIDController flywheel_TopPID, flywheel_ButtomPID;
    private RelativeEncoder flywheel_TopEncoder, flywheel_ButtomEncoder;
    

    public ShooterSubsystem() {
        flywheel_Top = new CANSparkMax(15, MotorType.kBrushless);
        flywheel_buttom = new CANSparkMax(16, MotorType.kBrushless);
        setup();
    }

    public void setup() {
        flywheel_Top.restoreFactoryDefaults();
        flywheel_buttom.restoreFactoryDefaults();

        flywheel_buttom.setInverted(true);

        flywheel_Top.setSmartCurrentLimit(40);
        flywheel_buttom.setSecondaryCurrentLimit(40);

        flywheel_TopPID = flywheel_Top.getPIDController();
        flywheel_ButtomPID = flywheel_buttom.getPIDController();

        flywheel_TopEncoder = flywheel_Top.getEncoder();
        flywheel_ButtomEncoder = flywheel_buttom.getEncoder();

        flywheel_TopEncoder.setVelocityConversionFactor((1 / (4 * Math.PI * 12)));
        flywheel_ButtomEncoder.setVelocityConversionFactor((1 / (4 * Math.PI * 12)));

        flywheel_TopPID.setP(Constants.PIDConstants.FLYWHEELTOP_PID0_P, 0);
        flywheel_TopPID.setI(Constants.PIDConstants.FLYWHEELTOP_PID0_I, 0);
        flywheel_TopPID.setD(Constants.PIDConstants.FLYWHEELTOP_PID0_D, 0);
        flywheel_TopPID.setFF(Constants.PIDConstants.FLYWHEELTOP_PID0_F, 0);

        flywheel_ButtomPID.setP(Constants.PIDConstants.FLYWHEELBUTTOM_PID0_P, 0);
        flywheel_ButtomPID.setI(Constants.PIDConstants.FLYWHEELBUTTOM_PID0_I, 0);
        flywheel_ButtomPID.setD(Constants.PIDConstants.FLYWHEELBUTTOM_PID0_D, 0);
        flywheel_ButtomPID.setFF(Constants.PIDConstants.FLYWHEELBUTTOM_PID0_F, 0);

        flywheel_Top.setIdleMode(IdleMode.kBrake);
        flywheel_buttom.setIdleMode(IdleMode.kBrake);

        flywheel_Top.burnFlash();
        flywheel_buttom.burnFlash();
    }

    public void setBrake(boolean mode){
        if(mode){
            flywheel_Top.setIdleMode(IdleMode.kBrake);
            flywheel_buttom.setIdleMode(IdleMode.kBrake);
        }else{
            flywheel_Top.setIdleMode(IdleMode.kCoast);
            flywheel_buttom.setIdleMode(IdleMode.kCoast);
        }
    }

    public void setReversed(boolean isReversed) {
        flywheel_Top.setInverted(isReversed);
        flywheel_buttom.setInverted(isReversed);
    }

    public void setSpeed(double velocity){
        if(Math.abs(velocity) > Constants.PhysicalConstants.FLYWHEEL_MAX_VELOCITY){
            velocity = Math.signum(velocity) * Constants.PhysicalConstants.FLYWHEEL_MAX_VELOCITY;
        };

        flywheel_TopPID.setReference(velocity, ControlType.kVelocity);
        flywheel_ButtomPID.setReference(velocity, ControlType.kVelocity);
    }

    public void setSpeedTop(double velocity){
        if(velocity > Constants.PhysicalConstants.FLYWHEEL_MAX_VELOCITY){
            velocity = Constants.PhysicalConstants.FLYWHEEL_MAX_VELOCITY;
        };
        
        flywheel_TopPID.setReference(velocity, ControlType.kVelocity);
    }

    public void setSpeedButtom(double velocity){
        if(velocity > Constants.PhysicalConstants.FLYWHEEL_MAX_VELOCITY){
            velocity = Constants.PhysicalConstants.FLYWHEEL_MAX_VELOCITY;
        };

        flywheel_ButtomPID.setReference(velocity, ControlType.kVelocity);
    }

    public void setTop(double speed) {
        flywheel_Top.set(speed);
    }

    public void setButtom(double speed) {
        flywheel_buttom.set(speed);
    }

    public double getVelocityTop(){
        return flywheel_TopEncoder.getVelocity();
    }

    public double getVelocityButtom(){
        return flywheel_ButtomEncoder.getVelocity();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("FLYWHEEL TOP", getVelocityTop());
        SmartDashboard.putNumber("FLYWHEEL BUTTOM", getVelocityButtom());
    }
}