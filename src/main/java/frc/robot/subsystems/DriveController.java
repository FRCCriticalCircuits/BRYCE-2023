package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;

public class DriveController extends SubsystemBase {
    private DriveSubsystem drive;
    private double frontleftangle, frontrightangle, rearleftangle, rearrightangle;
    private double frontleftspeed, frontrightspeed, rearleftspeed, rearrightspeed;
    private SlewRateLimiter limiter = new SlewRateLimiter(.5);
    private SwerveDriveKinematics kinematics = Constants.PhysicalConstants.KINEMATICS;
    private SwerveControllerCommand driveControllerCommand;

    public DriveController(DriveSubsystem drive) {
        this.drive = drive;
        setup();
    }

    public void setup(){
        
    }

    public void drive(double x1, double y, double x2, boolean fieldOrientedDrive) {
        new ChassisSpeeds();
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(
            !fieldOrientedDrive 
            ? new ChassisSpeeds(x1, y, x2) : 
            ChassisSpeeds.fromFieldRelativeSpeeds(x1, y, x2, drive.getHeading())
        );

        drive.setStates(moduleStates);
    }

    @Override
    public void periodic(){
        
    }
}