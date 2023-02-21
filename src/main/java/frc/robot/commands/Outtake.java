package frc.robot.commands;

import java.lang.reflect.InaccessibleObjectException;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;

public class Outtake extends CommandBase {
    private Intake intake;
    private Trigger trigger;

    public Outtake(Intake intake, Trigger trigger) {
        this.intake = intake;
        this.trigger = trigger;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        if(trigger.getAsBoolean()){
            intake.outake();
        }else{
            intake.stopIntake();
        }
    }

    @Override
    public void end(boolean interupted) {}

    @Override
    public boolean isFinished(){
        if(trigger.getAsBoolean()){
            return false;
        }else{
            return true;
        }
    }

}