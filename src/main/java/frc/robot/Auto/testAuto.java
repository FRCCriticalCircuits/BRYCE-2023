package frc.robot.Auto;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoSpinUp;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Sequencer;
import frc.robot.subsystems.ShooterSubsystem;

public class testAuto extends CommandBase{
    DriveSubsystem drive;
    Intake intake;
    ShooterSubsystem shooter;
    Sequencer sequencer;

    public testAuto(DriveSubsystem drive, Intake intake, ShooterSubsystem shooter, Sequencer sequencer) {
        this.drive = drive;
        this.intake = intake;
        this.shooter = shooter;
        this.sequencer = sequencer;

        addRequirements(drive);
    }

    public Command testAuto() {

        List<PathPlannerTrajectory> testTaxi = PathPlanner.loadPathGroup("Test Path", 1, 0.6);

        HashMap<String, Command> eventMAP = new HashMap<>();
        eventMAP.put("Marker 1", new PrintCommand("Marker 1")); 
        eventMAP.put("Shoot", new AutoSpinUp(shooter, sequencer, 1, 10));
        eventMAP.put("Intake", new AutoIntake(intake, sequencer));
        eventMAP.put("ShootSecond", new AutoSpinUp(shooter, sequencer, 1, 10));

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            drive::getPose, 
            drive::setPose, 
            new PIDConstants(
                0.4,
                0,
                0
                ),
            new PIDConstants(
                0.25, 
                0, 
                0), 
            drive::OutputChassisSpeeds, 
            eventMAP,
            true,
            drive
        );

        return new SequentialCommandGroup( 
            autoBuilder.fullAuto(testTaxi)
        );
    }
}
