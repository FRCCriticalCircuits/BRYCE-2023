package frc.robot.Util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveController extends SubsystemBase {
    private DriveSubsystem drive;
    private SlewRateLimiter limiter = new SlewRateLimiter(14);
    private SwerveDriveKinematics kinematics = Constants.PhysicalConstants.KINEMATICS;
    private SwerveControllerCommand driveControllerCommand;
    private PIDController controller = new PIDController(0.1, 0, 0);
    private double desiredheading, kXY = 0;

    public DriveController(DriveSubsystem drive) {
        this.drive = drive;
        setup();
    }

    public void setup(){
        desiredheading = drive.getPose().getRotation().getDegrees();
    }

    public void drive(double y, double x1, double x2, boolean fieldOrientedDrive) {
        x1 *= Constants.PhysicalConstants.MAX_METERS_PER_SECOND;
        x2 *= 3.75;
        y *= Constants.PhysicalConstants.MAX_METERS_PER_SECOND;
        
        //driftCorrect(x1, y, x2);

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(
            !fieldOrientedDrive 
            ? new ChassisSpeeds(x1, y, -x2) : 
            ChassisSpeeds.fromFieldRelativeSpeeds(x1, y, x2, drive.getRotation2d())
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.PhysicalConstants.MAX_METERS_PER_SECOND);
        drive.setStates(moduleStates);
    }

    public void driftCorrect(double x, double y, double x2) {
        double xy = Math.abs(x) + Math.abs(y);

        if(Math.abs(x2) > 0 || kXY <= 0.01){
            desiredheading = drive.getPose().getRotation().getDegrees();
        }else if(xy > 0){
            x2 += controller.calculate(drive.getPose().getRotation().getDegrees(), desiredheading);
        }

        kXY = xy;
    }

    @Override
    public void periodic(){
    }
}