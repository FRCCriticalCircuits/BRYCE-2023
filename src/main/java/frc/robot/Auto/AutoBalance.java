package frc.robot.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util.DriveController;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalance extends CommandBase {
    private DriveSubsystem drive;
    private DriveController driveController;
    private PIDController controller;
    private boolean isDropping = false;
    private int drops = 0;


    enum stage {
        CLIMBING, BALANCING
    }

    stage currentStage = stage.CLIMBING;


    public AutoBalance(DriveSubsystem drive, DriveController driveController) {
        this.drive = drive;
        this.driveController = driveController;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        controller = new PIDController(0.05, 0, 0);
        controller.setTolerance(1);
    }

    @Override
    public void execute() {
        driveController.drive(0, -controller.calculate(drive.getRoll(), 0), 0, false);
    }

    @Override
    public void end(boolean interrupted) {
        drive.frontLeft.setAngle(60);
        drive.frontRight.setAngle(-60);
        drive.rearLeft.setAngle(120);
        drive.rearRight.setAngle(-120);
    }

    @Override
    public boolean isFinished() {
        if(controller.atSetpoint()){
            return true;
        }else{
            return false;
        }
    }
    
}
