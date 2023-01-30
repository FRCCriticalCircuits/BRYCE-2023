package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private CANSparkMax intake;
    private SparkMaxPIDController intakePID;

    public Intake() {
        intake = new CANSparkMax(10, MotorType.kBrushless);
    }

    public void setup() {
        intake.restoreFactoryDefaults();

        intake.setSmartCurrentLimit(0);

        intakePID = intake.getPIDController();

        intake.burnFlash();
    }

    public void runIntake() {
        intake.set(.5);
    }

    public void stopIntake() {
        intake.setVoltage(0);
    }

    @Override
    public void periodic(){}
}