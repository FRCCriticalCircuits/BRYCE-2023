package frc.robot.Auto;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
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
import frc.robot.Constants.PIDConstants;
import frc.robot.Util.AutoPIDControllers;
import frc.robot.Util.DriveController;
import frc.robot.commands.AutoSpinUp;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Sequencer;
import frc.robot.subsystems.ShooterSubsystem;

public class MiddleAuto extends CommandBase{
    DriveSubsystem drive;
    Sequencer sequencer;
    ShooterSubsystem shooter;
    DriveController driveController;
    AutoPIDControllers autoPIDControllers;

    public MiddleAuto(DriveSubsystem drive, ShooterSubsystem shooter, Sequencer sequencer, DriveController driveController, AutoPIDControllers autoPIDControllers) {
        this.drive = drive;
        this.autoPIDControllers = autoPIDControllers;
        this.driveController = driveController;
        this.sequencer = sequencer;
        this.shooter = shooter;

        addRequirements(drive);
    }

    public Command middletaxi(){
        return new SequentialCommandGroup(
            new InstantCommand(drive::reset, drive),
            new AutoSpinUp(shooter, sequencer, 1, 10),
            new AutoBalanceBackup(drive, driveController)
        );
    }
    }