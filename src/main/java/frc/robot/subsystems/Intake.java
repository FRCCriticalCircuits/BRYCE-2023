package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private CANSparkMax intake1;
    private CANSparkMax intake2;
    private SparkMaxPIDController intake1PID, intake2PID;
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0);

    public Intake() {
        intake1 = new CANSparkMax(20, MotorType.kBrushless);
        intake2 = new CANSparkMax(21, MotorType.kBrushless);
    }

    public void setup() {
        intake1.restoreFactoryDefaults();
        intake2.restoreFactoryDefaults();

        intake1.setSmartCurrentLimit(20);
        intake2.setSmartCurrentLimit(20);

        intake1PID = intake1.getPIDController();
        intake2PID = intake2.getPIDController();

        intake1PID.setP(Constants.PIDConstants.INTAKE_PID0_P, 0);
        intake1PID.setI(Constants.PIDConstants.INTAKE_PID0_I, 0);
        intake1PID.setD(Constants.PIDConstants.INTAKE_PID0_D, 0);
        intake1PID.setFF(Constants.PIDConstants.INTAKE_PID0_F, 0);

        intake2PID.setP(Constants.PIDConstants.INTAKE_PID0_P, 0);
        intake2PID.setI(Constants.PIDConstants.INTAKE_PID0_I, 0);
        intake2PID.setD(Constants.PIDConstants.INTAKE_PID0_D, 0);
        intake2PID.setFF(Constants.PIDConstants.INTAKE_PID0_F, 0);

        intake1.setIdleMode(IdleMode.kCoast);
        intake2.setIdleMode(IdleMode.kCoast);

        intake1.burnFlash();
        intake2.burnFlash();
    }

    public void setIntakeSpeed(double speedPerSecond){
        double arb_ff = feedforward.calculate(speedPerSecond);
        intake1PID.setReference(speedPerSecond, ControlType.kVelocity, 0, arb_ff);
        intake2PID.setReference(speedPerSecond, ControlType.kVelocity, 0, arb_ff);

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
    public void periodic(){
        intake1.getEncoder().getVelocity();
        intake2.getEncoder().getVelocity();
    }
}