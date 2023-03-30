package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    private CANSparkMax window;
    private SparkMaxPIDController window_PID;
    private PIDController controller;
    private RelativeEncoder window_Encoder;
    private DoubleSolenoid coneManipulator = new DoubleSolenoid(PneumaticsModuleType.REVPH, 8, 9);
    private DutyCycleEncoder encoder = new DutyCycleEncoder(0);
    private ArmSubsystem instance;
    
    public ArmSubsystem() {
        window = new CANSparkMax(Constants.MotorIDs.ARM_ID, MotorType.kBrushed);
        setup();
    }

    public void setup() {
        window.restoreFactoryDefaults();

        window.setIdleMode(IdleMode.kBrake);

        //window_AbsoluteEncoder.setPositionConversionFactor((1 / 1.25) * 360);

        window.setSmartCurrentLimit(30);

        window.burnFlash();
    }

    public void setPosition(double angle) {
        window_PID.setReference(angle, ControlType.kPosition);
    }

    public void set(double forward){
        forward /= 0.3;

        window.set(forward);
    }

    public void setConeManipulator() {
        coneManipulator.set(Value.kForward);
    }

    public void engageConeManipulator() {
        coneManipulator.set(Value.kReverse);
    }



    public double getAngle() {
        return encoder.getAbsolutePosition();
    }

    public double getAbsoluteAngle() {
        return 0;
    }

    @Override
    public void periodic() {
        //SmartDashboard.putNumber("ARM ANGLE", getAngle());
        SmartDashboard.putNumber("ARM ABSOLUTE", getAbsoluteAngle());
    }
}
