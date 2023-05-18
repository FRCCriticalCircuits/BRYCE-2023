package frc.robot.Auto;

import java.util.HashMap;
import java.util.List;

import org.opencv.ml.DTrees;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoSpinUp;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Sequencer;
import frc.robot.subsystems.ShooterSubsystem;

public class SimpleOneCargoTaxi extends CommandBase {
    DriveSubsystem drive;
    ShooterSubsystem shooter;
    Sequencer sequencer;

    public SimpleOneCargoTaxi (DriveSubsystem drive, ShooterSubsystem shooter, Sequencer sequencer) {
        this.drive = drive;
        this.sequencer = sequencer;
        this.shooter = shooter;

        addRequirements(drive);
    }

    public Command simpleonecargotaxi() {
        List<PathPlannerTrajectory> simpleTaxi = PathPlanner.loadPathGroup("Simple Taxi", 2, 1.4);

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Shoot", new AutoSpinUp(shooter, sequencer, 1, 10));

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            drive::getPose, 
            drive::setPose, 
            new PIDConstants(
                0, 
                0, 
                0
            ), 
            new PIDConstants(
                0, 
                0, 
                0
            ), 
            drive::OutputChassisSpeeds, 
            eventMap, 
            drive
        );

        return new SequentialCommandGroup(
            new InstantCommand(drive::reset, drive),
            new AutoSpinUp(shooter, sequencer, 1, 8),
            autoBuilder.fullAuto(simpleTaxi),
            new InstantCommand(() -> drive.setGyroOffset(180), drive)
        );
    }
}