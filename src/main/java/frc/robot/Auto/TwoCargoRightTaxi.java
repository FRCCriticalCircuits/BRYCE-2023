package frc.robot.Auto;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Util.AutoPIDControllers;
import frc.robot.commands.AutoSpinUp;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Sequencer;
import frc.robot.subsystems.ShooterSubsystem;

public class TwoCargoRightTaxi extends CommandBase{
    DriveSubsystem drive;
    Sequencer sequencer;
    ShooterSubsystem shooter;
    Intake intake;
    AutoPIDControllers autoPIDControllers;
    //String route1 = "frc/paths/route1.wpilib.json";
    //Trajectory route = new Trajectory();

    public TwoCargoRightTaxi(DriveSubsystem drive, ShooterSubsystem shooter, Intake intake, Sequencer sequencer, AutoPIDControllers autoPIDControllers) {
        this.drive = drive;
        this.autoPIDControllers = autoPIDControllers;
        this.intake = intake;
        this.sequencer = sequencer;
        this.shooter = shooter;

        addRequirements(drive);
    }

    public Command cargorighttaxi(){
        List<PathPlannerTrajectory> rightTaxi = PathPlanner.loadPathGroup("Right Taxi", 1.2, 0.7);
        
        HashMap<String, Command> eventMap = new HashMap<>();

        eventMap.put("MARKER 1", new PrintCommand("PASSED MARKER 1"));

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
                0
            ), 
            drive::OutputChassisSpeeds, 
            eventMap, 
            drive
        );

        return new SequentialCommandGroup(
            new AutoSpinUp(shooter, sequencer, 0, 1.5, 13),
            autoBuilder.fullAuto(rightTaxi),
            new InstantCommand(drive::resetHeading, drive)
        );
    }
    
}
