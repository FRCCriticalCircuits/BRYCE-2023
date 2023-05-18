package frc.robot.Auto;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Util.DriveController;
import frc.robot.commands.AutoSpinUp;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Sequencer;
import frc.robot.subsystems.ShooterSubsystem;

public class Mobility extends CommandBase {
    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private Sequencer sequencer;
    private DriveController driveController;
    private LimelightSubsystem limelight;

    public Mobility(DriveSubsystem drive, ShooterSubsystem shooter, Sequencer sequencer, DriveController driveController) {
        this.drive = drive;
        this.shooter = shooter;
        this.sequencer = sequencer;
        this.driveController = driveController;
        
        addRequirements(drive);
    }

    public CommandBase mobility() {
        List<PathPlannerTrajectory> mobility = PathPlanner.loadPathGroup("Mobility", 1.5, 1);

        HashMap<String, Command> eventMap = new HashMap<>();

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            drive::getPose, 
            drive::setPose, 
            new PIDConstants(
                0.4, 
                0, 
                0
            ), 
            new PIDConstants(
                -0.25, 
                0, 
                0
            ), 
            drive::OutputChassisSpeeds, 
            eventMap,
            true,
            drive
        );

        return new SequentialCommandGroup(
            new InstantCommand(drive::reset, drive),
            new AutoSpinUp(shooter, sequencer,1 , 8),
            autoBuilder.fullAuto(mobility),
            new AutoBalance(drive, driveController, true),
            new InstantCommand(() -> drive.setGyroOffset(180), drive)  
        );
    }
    
}
