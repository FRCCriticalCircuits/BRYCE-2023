package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Sequencer;

public class RunSequencer extends CommandBase {
    private Sequencer sequencer;
    private Trigger trigger;
    private boolean isReversed;
    private double time = 0;
    private double startDelta;

    /*
        CONSTRUCTOR FOR COMMAND
        
        RUNS THE SEQUENCER FOR A NUMBER OF SECONDS

        RECOMMENDED FOR TELEOP
    */
    public RunSequencer(Sequencer sequencer, boolean isReversed, Trigger trigger) {
        this.sequencer = sequencer;
        this.isReversed = isReversed;
        this.trigger = trigger;

        addRequirements(sequencer);
    }

    /*
        CONSTRUCTOR FOR COMMAND
        
        RUNS THE SEQUENCER FOR A NUMBER OF SECONDS

        RECOMMENDED FOR AUTONOMOUS
    */
    public RunSequencer(Sequencer sequencer, boolean isReversed, double time) {
        this.sequencer = sequencer;
        this.isReversed = isReversed;
        this.time = time;

        addRequirements(sequencer);
    }

    @Override
    public void initialize() {
        startDelta = Timer.getFPGATimestamp();
    }
    
    @Override
    public void execute() {
        if(trigger.getAsBoolean() || (Timer.getFPGATimestamp() - startDelta) < time){
            sequencer.run(isReversed);
        }
    }

    @Override
    public void end(boolean isFinished){
        sequencer.stop();
    }

    @Override
    public boolean isFinished() {
        if(!trigger.getAsBoolean() && (Timer.getFPGATimestamp() - startDelta) > time){
            return true;
        }else{
            return false;
        }
    }
}