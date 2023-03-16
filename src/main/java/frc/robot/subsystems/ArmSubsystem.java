package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    private CANSparkMax window;
    private SparkMaxPIDController window_PID;
    private RelativeEncoder window_Encoder;
    private ArmSubsystem instance;

    public ArmSubsystem() {
        setup();
    }

    public void getInstance() {
        instance = new ArmSubsystem();
    }

    public void setup() {
        window.restoreFactoryDefaults();

        window = new CANSparkMax(Constants.MotorIDs.ARM_ID, MotorType.kBrushed);

        window_PID = window.getPIDController();
        window_PID.setOutputRange(-270, 270);

        window_Encoder = window.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 8192);
        window_Encoder.setPosition(0);

        window_Encoder.setPositionConversionFactor(360);

        window_PID.setFeedbackDevice(window_Encoder);

        window.setSmartCurrentLimit(20);

        window.burnFlash();
    }

    public void setPosition(double angle) {
        window_PID.setReference(angle, ControlType.kPosition);
    }

    public void set(double forward){
        forward /= 0.2;

        window.set(forward);
    }

    public double getAngle() {
        return window_Encoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ARM ANGLE", getAngle());
    }
}
