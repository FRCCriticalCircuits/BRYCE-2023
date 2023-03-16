package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinUp extends CommandBase {
    public ShooterSubsystem shooter;
    public double velocity, time, startDelta, percentSpin = 0;
    public boolean isSpin;
    public Trigger trigger;

    /**
     * SPIN UP THE SHOOTER
     * 
     * @param shooter shooter subsystem object
     * @param velocity net velocity of shooter
     * @param Trigger trigger for command 
     */
    public SpinUp(ShooterSubsystem shooter, double velocity, Trigger trigger) {
        this.shooter = shooter;
        this.velocity = velocity;
        this.trigger = trigger;

        addRequirements(shooter);
    }

    /**
     * SPINS UP THE FLYWHEEL TO NEEDED VELOCITY
     * 
     * @param shooter shooter subsystem
    */
    public SpinUp(ShooterSubsystem shooter, double velocity, double time) {
        this.shooter = shooter;
        this.velocity = velocity;
        this.time = time;

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
    public SpinUp(ShooterSubsystem shooter, double velocity, double percentSpin, Trigger trigger){
        this.shooter = shooter;
        this.velocity = velocity;
        this.percentSpin = percentSpin;
        this.trigger = trigger;
        
        addRequirements(shooter);
    }

    /*
     * SPINS UP THE FLYWHEEL TO NEEDED VELOCITY
     *
     * POSITIVE PERCENT SPIN WILL CAUSE BACKSPIN
     *   NEGATIVE PERCENT SPIN WILL CAUSE FORWARD SPIN
    */
    public SpinUp(ShooterSubsystem shooter, double velocity, double percentSpin, double time){
        this.shooter = shooter;
        this.velocity = velocity;
        this.percentSpin = percentSpin;
        this.time = time;
        
        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {
        startDelta = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if(trigger.getAsBoolean() || (Timer.getFPGATimestamp() - startDelta) < time){
            if(percentSpin > 0){
                shooter.setSpeedTop(velocity + (velocity * (percentSpin / 2)));
                shooter.setSpeed(velocity + -(velocity * (percentSpin / 2)));
            }else{
                shooter.setSpeed(velocity);
            }
        }
    }

    @Override
    public boolean isFinished() {
        if(!trigger.getAsBoolean() && (Timer.getFPGATimestamp() - startDelta) > time){
            return true;
        }else{
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setSpeed(0);
    }
}
