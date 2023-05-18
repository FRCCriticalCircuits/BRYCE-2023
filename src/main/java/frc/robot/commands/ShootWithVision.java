package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Util.GoalType.goalType;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Sequencer;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootWithVision extends CommandBase{
    private ShooterSubsystem shooter;
    private Sequencer sequencer;
    private LimelightSubsystem limelight;
    private double velocity = 0, percentSpin = 0;
    private Trigger trigger;

    public ShootWithVision(ShooterSubsystem shooter, Sequencer sequencer, LimelightSubsystem limelight) {
        this.shooter = shooter;
        this.sequencer = sequencer;
        this.limelight = limelight;

        addRequirements(shooter, limelight);
    }

    public ShootWithVision(ShooterSubsystem shooter, Sequencer sequencer, LimelightSubsystem limelight, Trigger trigger) {
        this.shooter = shooter;
        this.sequencer = sequencer;
        this.trigger = trigger;
        this.limelight = limelight;

        addRequirements(shooter, limelight);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        
        if(limelight.getCurrentGoal() == goalType.MID){
            //velocity = Math.pow(7.222, 0.1922 * limelight.getGoalDistance());
            velocity = ((2.665) * limelight.getGoalDistance() + 6.905) - 1.25;
        }else if(limelight.getCurrentGoal() == goalType.HIGH){
            velocity = ((2.665) * limelight.getGoalDistance() + 6.905) - 0.4;
        }else if(limelight.getCurrentGoal() == goalType.LOW){
            velocity = 10;
        }else{
            velocity = 0;
        }
        
        if(percentSpin > 0){
            shooter.setSpeedTop(velocity + (velocity * (percentSpin / 2)));
            shooter.setSpeedButtom(velocity + -(velocity * (percentSpin / 2)));
        }else{
            shooter.setSpeed(velocity);
        }

        sequencer.run(0.4, false);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setSpeed(0);
        sequencer.stop();
    }

    @Override
    public boolean isFinished() {
        if(!trigger.getAsBoolean()){
            return true;
        }else{
            return false;
        }
    }
}