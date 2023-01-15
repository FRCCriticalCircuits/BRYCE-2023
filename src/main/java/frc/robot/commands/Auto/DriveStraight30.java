package frc.robot.commands.Auto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive.DriveSubsystem;

public class DriveStraight30 extends CommandBase {
    private DriveSubsystem drive;


    public DriveStraight30(DriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.frontLeft.resetEncoderForward();
        drive.frontRight.resetEncoderForward();
        drive.rearLeft.resetEncoderForward();
        drive.rearRight.resetEncoderForward();

        SmartDashboard.getNumber("FRONT LEFT DISTANCE", drive.frontLeft.getDistance());
        SmartDashboard.getNumber("FRONT RIGHT DISTANCE", drive.frontRight.getDistance());
        SmartDashboard.getNumber("REAR LEFT DISTANCE", drive.rearLeft.getDistance());
        SmartDashboard.getNumber("REAR RIGHT DISTANCE", drive.rearRight.getDistance());

        SmartDashboard.getNumber("TARGET", Units.metersToInches(3));
    }

    @Override
    public void execute() {
        drive.frontLeft.moveToInches(Units.metersToInches(3));
        drive.frontRight.moveToInches(Units.metersToInches(3));
        drive.rearLeft.moveToInches(Units.metersToInches(3));
        drive.rearRight.moveToInches(Units.metersToInches(3));
    }

}