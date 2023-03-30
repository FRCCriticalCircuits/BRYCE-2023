package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ArmSubsystem;

public class ManualArmControl extends CommandBase{
    double percentOutput;
    ArmSubsystem armSubsystem;
    Joystick joystick = new Joystick(1);
    SlewRateLimiter limiter = new SlewRateLimiter(1.675);

    public ManualArmControl(ArmSubsystem armSubsystem, double percentOutput) {
        this.armSubsystem = armSubsystem;
        this.percentOutput = percentOutput;

        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        percentOutput = joystick.getRawAxis(1);

        armSubsystem.set(limiter.calculate(percentOutput));
    }
    
    @Override
    public boolean isFinished() {
        if(Math.abs(percentOutput) > 0.05){
            return true;
        }else{
            return false;
        }
    }
}
