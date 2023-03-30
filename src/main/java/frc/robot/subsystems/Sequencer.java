package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Sequencer extends SubsystemBase{
    private CANSparkMax sequencer = new CANSparkMax(Constants.MotorIDs.SEQUENCER_ID, MotorType.kBrushless);        

    public Sequencer(){
        sequencer.setInverted(Constants.MotorIDs.SEQUENCER_ISREVERSED);
    }

    public void run(boolean isReversed) {
        if(!isReversed){
            sequencer.set(.6);
        }else{
            sequencer.set(-.4);
        }
    }
    
    public void run(double percentOutput, boolean isReversed) {
        if(!isReversed){
            sequencer.set(percentOutput);
        }else{
            sequencer.set(-percentOutput);
        }
    }

    public void stop() {
        sequencer.setVoltage(0);
    }
}
