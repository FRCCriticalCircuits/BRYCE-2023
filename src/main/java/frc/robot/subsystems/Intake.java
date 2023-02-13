package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private CANSparkMax intake1;
    private CANSparkMax intake2;
    private SparkMaxPIDController intakePID;

    public Intake() {
        intake1 = new CANSparkMax(20, MotorType.kBrushless);
        intake2 = new CANSparkMax(21, MotorType.kBrushless);
    }

    public void setup() {
        intake1.restoreFactoryDefaults();
        intake2.restoreFactoryDefaults();

        intake1.setSmartCurrentLimit(0);

        intakePID = intake1.getPIDController();

        intake1.setIdleMode(IdleMode.kCoast);
        intake2.setIdleMode(IdleMode.kCoast);

        intake1.burnFlash();
        intake2.burnFlash();
    }

    public void runIntake() {
        intake1.set(.5);
        intake2.set(.5);
    }

    public void stopIntake() {
        intake1.setVoltage(0);
        intake2.setVoltage(0);
    }

    @Override
    public void periodic(){}
}