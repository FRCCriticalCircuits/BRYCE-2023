package frc.robot.subsystems;

import javax.swing.plaf.basic.BasicTreeUI.TreeCancelEditingAction;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private CANSparkMax Intake_Left;
    private CANSparkMax Intake_Right;
    private SparkMaxPIDController intake1PID, intake2PID;
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0);

    public Intake() {
        Intake_Left = new CANSparkMax(20, MotorType.kBrushless);
        Intake_Right = new CANSparkMax(21, MotorType.kBrushless);
    }

    public void setup() {
        Intake_Left.restoreFactoryDefaults();
        Intake_Right.restoreFactoryDefaults();

        Intake_Left.setSmartCurrentLimit(20);
        Intake_Right.setSmartCurrentLimit(20);

        Intake_Right.setInverted(true);
        Intake_Left.setInverted(false);

        intake1PID = Intake_Left.getPIDController();
        intake2PID = Intake_Right.getPIDController();

        intake1PID.setP(Constants.PIDConstants.INTAKE_PID0_P, 0);
        intake1PID.setI(Constants.PIDConstants.INTAKE_PID0_I, 0);
        intake1PID.setD(Constants.PIDConstants.INTAKE_PID0_D, 0);
        intake1PID.setFF(Constants.PIDConstants.INTAKE_PID0_F, 0);

        intake2PID.setP(Constants.PIDConstants.INTAKE_PID0_P, 0);
        intake2PID.setI(Constants.PIDConstants.INTAKE_PID0_I, 0);
        intake2PID.setD(Constants.PIDConstants.INTAKE_PID0_D, 0);
        intake2PID.setFF(Constants.PIDConstants.INTAKE_PID0_F, 0);

        Intake_Left.setIdleMode(IdleMode.kCoast);
        Intake_Right.setIdleMode(IdleMode.kCoast);

        Intake_Left.burnFlash();
        Intake_Right.burnFlash();
    }

    public void setIntakeSpeed(double speedPerSecond){
        double arb_ff = feedforward.calculate(speedPerSecond);
        intake1PID.setReference(speedPerSecond, ControlType.kVelocity, 0, arb_ff);
        intake2PID.setReference(speedPerSecond, ControlType.kVelocity, 0, arb_ff);

    }

    public void runIntake() {
        Intake_Left.set(.5);
        Intake_Right.set(-.5);
    }

    public void outake() {
        Intake_Left.set(-0.3);
        Intake_Right.set(0.3);
    }

    public void stopIntake() {
        Intake_Left.setVoltage(0);
        Intake_Right.setVoltage(0);
    }

    @Override
    public void periodic(){
        Intake_Left.getEncoder().getVelocity();
        Intake_Right.getEncoder().getVelocity();
    }
}