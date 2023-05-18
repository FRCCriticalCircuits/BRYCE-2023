package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Util.AutoPIDControllers;
import frc.robot.Util.DriveController;
import frc.robot.Util.GoalType.goalType;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AlignToTarget extends CommandBase {
    private DriveSubsystem drive;
    private DriveController driveController;
    private LimelightSubsystem limelight;
    private boolean isAutonomous;
    private PIDController controller;
    private Trigger trigger;
    private DoubleSupplier _trigger;

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
    public void initialize() {
        controller = new PIDController(0.018, 0, 0.002);
        //controller.setIntegratorRange(1, 1);
    }

    @Override
    public void execute() {
        double x1 = drive.driverJoystick.getLeftX();
        double y = drive.driverJoystick.getLeftY();

        if(Math.abs(x1) < 0.15) {
            x1 = 0;
        }

        if(Math.abs(y) < 0.15) {
            y = 0;
        }

        driveController.drive(x1, y, controller.calculate(limelight.getXOffset(), -limelight.getBestGoalOffset()), true);
    }

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
