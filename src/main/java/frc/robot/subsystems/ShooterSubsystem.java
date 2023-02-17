package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private CANSparkMax flywheel_Top, flywheel_buttom;
    private SparkMaxPIDController flywheel_TopPID, flywheel_ButtomPID;
    private RelativeEncoder flywheel_TopEncoder, flywheel_ButtomEncoder;

    public ShooterSubsystem() {
        setup();
    }

    public void setup() {
        flywheel_Top.restoreFactoryDefaults();
        flywheel_buttom.restoreFactoryDefaults();

        flywheel_Top = new CANSparkMax(30, MotorType.kBrushless);
        flywheel_buttom = new CANSparkMax(31, MotorType.kBrushless);

        flywheel_TopPID = flywheel_Top.getPIDController();
        flywheel_ButtomPID = flywheel_buttom.getPIDController();

        flywheel_TopEncoder = flywheel_Top.getEncoder();
        flywheel_ButtomEncoder = flywheel_buttom.getEncoder();

        flywheel_TopPID.setP(0, 0);
        flywheel_TopPID.setI(0, 0);
        flywheel_TopPID.setD(0, 0);
        flywheel_TopPID.setFF(0, 0);

        flywheel_ButtomPID.setP(0, 0);
        flywheel_ButtomPID.setI(0, 0);
        flywheel_ButtomPID.setD(0, 0);
        flywheel_ButtomPID.setFF(0, 0);

        flywheel_Top.setIdleMode(IdleMode.kBrake);
        flywheel_buttom.setIdleMode(IdleMode.kBrake);

        flywheel_Top.burnFlash();
        flywheel_buttom.burnFlash();
    }



    @Override
    public void periodic(){

    }
    
}
