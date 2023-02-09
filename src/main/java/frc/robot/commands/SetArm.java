package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class SetArm extends CommandBase{
    private ArmSubsystem arm;

    public SetArm(ArmSubsystem arm) {
        this.arm = arm;

        addRequirements(arm);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
