package frc.robot.commands;

import java.security.PublicKey;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Sequencer;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinUp extends CommandBase {
    public ShooterSubsystem shooter;
    public Sequencer sequencer;
    public double velocity, delayStartDelta, delay, time, startDelta, percentSpin = 0;
    public boolean isSpin;
    public Trigger trigger;

    /**
     * SPIN UP THE SHOOTER
     * 
     * @param shooter shooter subsystem object
     * @param velocity net velocity of shooter
     * @param Trigger trigger for command 
     */
    public SpinUp(ShooterSubsystem shooter, Sequencer sequencer, double velocity, Trigger trigger) {
        this.shooter = shooter;
        this.velocity = velocity;
        this.trigger = trigger;
        this.sequencer = sequencer;

        addRequirements(shooter);
    }

    /**
     * SPINS UP THE FLYWHEEL TO NEEDED VELOCITY
     *
     * @param shooter shooter subsystem object
     * @param velocity net velocity of shooter
     * @param percentSpin percentage of backpin
     * @param Trigger trigger for command 
     * 
     * @return Positive percent spin would result in backspin, negative would result in forward spin
    */
    public SpinUp(ShooterSubsystem shooter, Sequencer sequencer, double velocity, double percentSpin, Trigger trigger){
        this.shooter = shooter;
        this.velocity = velocity;
        this.percentSpin = percentSpin;
        this.trigger = trigger;
        this.sequencer = sequencer;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        startDelta = Timer.getFPGATimestamp();
        delayStartDelta = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        delay = ((velocity * 5) / 80) * .5;
        
        if(trigger.getAsBoolean() || (Timer.getFPGATimestamp() - startDelta) < time){
            if(Math.abs(percentSpin) > 0){
                shooter.setSpeedTop(velocity + (velocity * (percentSpin / 2)));
                shooter.setSpeedButtom(velocity + -(velocity * (percentSpin / 2)));
            }else{
                shooter.setSpeed(velocity);
            }
        }

        if((Timer.getFPGATimestamp() - delayStartDelta) > delay){
            sequencer.run(0.4, false);
        }
    }

    @Override
    public boolean isFinished() {
        if((!trigger.getAsBoolean()) && (Timer.getFPGATimestamp() - startDelta) > time){
            return true;
        }else{
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setSpeed(0);
        sequencer.stop();
    }
}
