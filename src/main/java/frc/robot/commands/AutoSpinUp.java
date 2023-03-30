package frc.robot.commands;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Sequencer;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoSpinUp extends CommandBase {
    ShooterSubsystem shooter;
    Sequencer sequencer;
    double time = 1, startDelta, velocity, delay, percentSpin;
    boolean timeUP = false;

    public AutoSpinUp(ShooterSubsystem shooter, Sequencer sequencer, double velocity){
        this.shooter = shooter;
        this.sequencer = sequencer;
        this.velocity = velocity;

        addRequirements(shooter);
    }

    public AutoSpinUp(ShooterSubsystem shooter, double percentSpin, Sequencer sequencer, double velocity){
        this.shooter = shooter;
        this.sequencer = sequencer;
        this.velocity = velocity;
        this.percentSpin = percentSpin;

        addRequirements(shooter);
    }

    public AutoSpinUp(ShooterSubsystem shooter, Sequencer sequencer, double time, double velocity) {
        this.shooter = shooter;
        this.sequencer = sequencer;
        this.time = time;
        this.velocity = velocity;

        addRequirements(shooter);
    }

    public AutoSpinUp(ShooterSubsystem shooter, Sequencer sequencer, double percentSpin, double time, double velocity) {
        this.shooter = shooter;
        this.sequencer = sequencer;
        this.percentSpin = percentSpin;
        this.time = time;
        this.velocity = velocity;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        startDelta = Timer.getFPGATimestamp();

        new Thread(
            () -> {
                try {
                    Thread.sleep((long) (time * 1000));
                    timeUP = true;
                } catch (Exception e) {
                }
            }
        ).start();
    }

    @Override
    public void execute() {
        delay = ((velocity * 5) / 80) * .1;

        if(percentSpin > 0){
            shooter.setSpeedTop(velocity + (velocity * (percentSpin / 2)));
            shooter.setSpeedButtom(velocity + -(velocity * (percentSpin / 2)));
        }else{
            shooter.setSpeed(velocity);
        }

        if((Timer.getFPGATimestamp() - startDelta) > delay){
            sequencer.run(0.3,false);
        }
    }

    @Override
    public void end(boolean Interrupted){
        shooter.setSpeed(0);
        sequencer.stop();
    }

    @Override
    public boolean isFinished(){
        if(timeUP){
            return true;
        }else{
            return false;
        }
    }

}
