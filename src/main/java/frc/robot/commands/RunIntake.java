package frc.robot.commands;

import java.lang.reflect.InaccessibleObjectException;
import java.sql.Time;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {
    private Intake intake;
    private Trigger trigger;
    private boolean isReversed;
    private double startDelta, time = 0;
    
    public RunIntake(Intake intake, boolean isReversed, Trigger trigger) {
        this.intake = intake;
        this.trigger = trigger;
        this.isReversed = isReversed;

        addRequirements(intake);
    }

    public RunIntake(Intake intake, boolean isReversed, double time){
        this.intake = intake;
        this.isReversed = isReversed;
        this.time = time;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        startDelta = Timer.getFPGATimestamp();
    }
    

    @Override
    public void execute() {
        if(trigger.getAsBoolean() || (Timer.getFPGATimestamp() - startDelta) > time){
            if(!isReversed) {
                intake.runIntake();
            }else{
                intake.outake();
            }
        }
    }

    @Override
    public void end(boolean interupted) {
        intake.stopIntake();
    }

    @Override
    public boolean isFinished(){
        if(!trigger.getAsBoolean() && (Timer.getFPGATimestamp() - startDelta) > time){
            return true;
        }else{
            return false;
        }
    }

}