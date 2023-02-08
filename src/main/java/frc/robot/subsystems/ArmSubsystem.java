package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private CANSparkMax window = new CANSparkMax(15, MotorType.kBrushed);
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

        window_PID = window.getPIDController();
        window_PID.setOutputRange(-270, 270);

        window_Encoder = window.getAlternateEncoder(Type.kQuadrature , 8192);
        window_Encoder.setPosition(0);

        window_Encoder.setPositionConversionFactor(360);

        window.setSmartCurrentLimit(20);

        window.burnFlash();
    }

    public void setPosition(double angle) {
        window_PID.setReference(angle, ControlType.kPosition);
    }

    public void moveArm(double forward){
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
