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

    @Override
    public void periodic(){}
}
