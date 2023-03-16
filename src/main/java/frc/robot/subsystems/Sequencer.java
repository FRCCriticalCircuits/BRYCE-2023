package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Sequencer extends SubsystemBase{
    private Spark sequencer = new Spark(Constants.MotorIDs.SEQUENCER_PWM_ID);        

    public Sequencer(){
        sequencer.setSafetyEnabled(true);
        sequencer.setInverted(Constants.MotorIDs.SEQUENCER_ISREVERSED);
    }

    public void run(boolean isReversed) {
        if(!isReversed){
            sequencer.set(.6);
        }else{
            sequencer.set(-.4);
        }

        sequencer.feed();
    }
    
    public void stop() {
        sequencer.setVoltage(0);
    }
}
