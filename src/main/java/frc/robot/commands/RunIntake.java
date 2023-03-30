package frc.robot.commands;

import java.lang.reflect.InaccessibleObjectException;
import java.sql.Time;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Sequencer;

public class RunIntake extends CommandBase {
    private Intake intake;
    private Trigger _trigger;
    private DoubleSupplier trigger;
    private Sequencer sequencer;
    private boolean isReversed;
    private double startDelta, time = 0;
    
    public RunIntake(Intake intake, Sequencer sequencer, boolean isReversed, Trigger trigger) {
        this.intake = intake;
        this._trigger = trigger;
        this.isReversed = isReversed;
        this.sequencer = sequencer;

        addRequirements(intake);
    }

    public RunIntake(Intake intake, Sequencer sequencer, boolean isReversed, DoubleSupplier trigger) {
        this.intake = intake;
        this.trigger = trigger;
        this.sequencer = sequencer;
        this.isReversed = isReversed;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
    }
    

    @Override
    public void execute() {
        if(!isReversed) {
            intake.runIntake();
        }else{
            intake.outake();
        }

        sequencer.run(0.3 , true);
    }

    @Override
    public void end(boolean interupted) {
        intake.stopIntake();
        sequencer.stop();
    }

    @Override
    public boolean isFinished(){
        if(!_trigger.getAsBoolean()){
            return true;
        }else{
            return false;
        }
    }

}