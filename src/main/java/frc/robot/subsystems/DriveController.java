package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;

public class DriveController extends SubsystemBase {
    private DriveSubsystem drive;
    private SlewRateLimiter limiter = new SlewRateLimiter(16);
    private SwerveDriveKinematics kinematics = Constants.PhysicalConstants.KINEMATICS;
    private SwerveControllerCommand driveControllerCommand;
    private PIDController controller = new PIDController(0.004, 0, 0);
    private double kXY = 0;

    public DriveController(DriveSubsystem drive) {
        this.drive = drive;
        setup();
    }

    public void setup(){
        
    }

    public void drive(double x1, double y, double x2, boolean fieldOrientedDrive) {
        x1 *= 13;
        x2 *= 10;
        y *= -13;

        driftCorrect(x1, y, x2);

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(
            !fieldOrientedDrive 
            ? new ChassisSpeeds(y, x1, x2) : 
            ChassisSpeeds.fromFieldRelativeSpeeds(y, x1, x2, drive.getHeading())
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, 13);
        drive.setStates(moduleStates);
    }

    public void driftCorrect(double x, double y, double x2) {
        double xy = Math.abs(x) + Math.abs(y);
        double desiredheading = 0;

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