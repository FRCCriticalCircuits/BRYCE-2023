package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveController extends SubsystemBase {
    private DriveSubsystem drive;
    
    public DriveController(DriveSubsystem drive) {
        this.drive = drive;
        setup();
    }

    public void setup(){
        
    }

    public void translate(double angle, double power) {
        drive.frontLeft.setAngle(angle);
        drive.frontRight.setAngle(angle);
        drive.rearLeft.setAngle(angle);
        drive.rearRight.setAngle(angle);

        drive.frontLeft.setSpeed(power);
        drive.frontRight.setSpeed(power);
        drive.rearLeft.setSpeed(power);
        drive.rearRight.setSpeed(power);
    }

    @Override
    public void periodic(){}
}
