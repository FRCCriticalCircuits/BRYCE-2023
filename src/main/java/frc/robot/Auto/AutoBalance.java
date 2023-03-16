package frc.robot.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveController;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalance extends CommandBase {
    private DriveSubsystem drive;
    private DriveController driveController;
    private PIDController controller;

    public AutoBalance(DriveSubsystem drive, DriveController driveController) {
        this.drive = drive;
        this.driveController = driveController;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        controller = new PIDController(0, 0, 0);
    }

    @Override
    public void execute() {
        driveController.drive(0, 0, 0, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
