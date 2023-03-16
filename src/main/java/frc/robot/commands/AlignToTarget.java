package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Util.AutoPIDControllers;
import frc.robot.subsystems.DriveController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AlignToTarget extends CommandBase {
    private DriveSubsystem drive;
    private DriveController driveController;
    private LimelightSubsystem limelight;
    private boolean isAutonomous;
    private PIDController controller = new PIDController(0, 0, 0);
    private Trigger trigger;

    public AlignToTarget(DriveSubsystem drive, DriveController driveController, LimelightSubsystem limelight, boolean isAutonomous){
        this.drive = drive;
        this.isAutonomous = isAutonomous;
        this.driveController = driveController;
        this.limelight = limelight;

        addRequirements(drive);
    }

    public AlignToTarget(DriveSubsystem drive, DriveController driveController, LimelightSubsystem limelight, Trigger trigger){
        this.drive = drive;
        this.trigger = trigger;
        this.driveController = driveController;
        this.limelight = limelight;

        addRequirements(drive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        if(isAutonomous){
            if(controller.atSetpoint()){
                return true;
            }else{
                return false;
            }
        }else{
            if(trigger.getAsBoolean()){
                return false;
            }else{
                return true;
            }
        }
    }

}
