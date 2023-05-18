package frc.robot.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Util.DriveController;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalanceBackup extends CommandBase {
    private DriveSubsystem drive;
    private DriveController driveController;
    private PIDController controller;
    private double Output;
    private int state = 0;

    public AutoBalanceBackup(DriveSubsystem drive, DriveController driveController) {
        this.drive = drive;
        this.driveController = driveController;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.setBrakeMode(true);
        controller = new PIDController(0.0065, 0, 0);
    }

    @Override
    public void execute() {
        if(state == 0){
            driveController.drive(0, 0.2, 0, false);
        }
        
        if(state > 0){
            
            Output = controller.calculate(drive.getRoll(), 0);
            if(Math.abs(Output) > 0.18)
                Output = Math.signum(Output) * 0.15;
                driveController.drive(0, -controller.calculate(drive.getRoll(), 0), 0, false);
            

            //drive.stop();
        }

        if(Math.abs(drive.getRoll()) > 14){
            state++;
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.baselock();
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
