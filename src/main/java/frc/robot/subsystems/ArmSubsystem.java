package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private CANSparkMax window = new CANSparkMax(15, MotorType.kBrushed);
    private SparkMaxPIDController window_PID;

    public ArmSubsystem() {

    }

    public void setup() {
        window.restoreFactoryDefaults();

        window_PID = window.getPIDController();

        window.setSmartCurrentLimit(20);

        window.burnFlash();
    }

    @Override
    public void periodic() {}
}
