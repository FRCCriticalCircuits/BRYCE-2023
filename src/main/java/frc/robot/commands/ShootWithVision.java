package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util.GoalType.goalType;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Sequencer;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootWithVision extends CommandBase{
    private ShooterSubsystem shooter;
    private Sequencer sequencer;
    private LimelightSubsystem limelight;
    private double velocity = 0, time = 0, percentSpin = 0;
    private boolean TimeOver = false;

    public ShootWithVision(ShooterSubsystem shooter, Sequencer sequencer, LimelightSubsystem limelight) {
        this.shooter = shooter;
        this.sequencer = sequencer;
        this.limelight = limelight;

        addRequirements(shooter, limelight);
    }

    public ShootWithVision(ShooterSubsystem shooter, Sequencer sequencer, LimelightSubsystem limelight, double time) {
        this.shooter = shooter;
        this.sequencer = sequencer;
        this.limelight = limelight;
        this.time = time;

        addRequirements(shooter, limelight);
    }

    @Override
    public void initialize() {
        new Thread(
            () -> {
                try {
                    Thread.sleep((int) time * 1000);
                    TimeOver = true;
                } catch (Exception e) {
                }
            }
        ).start();
    }

    @Override
    public void execute() {
        
        if(limelight.getCurrentGoal() == goalType.MID){
            velocity = Math.pow(7.222, 0.1922 * limelight.getGoalDistance());
            //velocity = (2.665) * limelight.getGoalDistance() + 5.905;
        }else{
            velocity = 0;
        }

        if(percentSpin > 0){
            shooter.setSpeedTop(velocity + (velocity * (percentSpin / 2)));
            shooter.setSpeed(velocity + -(velocity * (percentSpin / 2)));
        }else{
            shooter.setSpeed(velocity);
        }

        sequencer.run(0.3, false);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setSpeed(0);
        sequencer.stop();
    }

    @Override
    public boolean isFinished() {
        if(TimeOver) {
            return true;
        }else{
            return false;
        }
    }
}