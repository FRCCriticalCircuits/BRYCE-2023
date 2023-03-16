package frc.robot.Auto;

import java.io.IOException;
import java.nio.file.Path;
import java.time.Duration;
import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Util.AutoPIDControllers;
import frc.robot.subsystems.DriveSubsystem;

public class TwoCargoRightTaxi extends CommandBase{
    DriveSubsystem drive;
    AutoPIDControllers autoPIDControllers;
    //String route1 = "frc/paths/route1.wpilib.json";
    //Trajectory route = new Trajectory();

    public TwoCargoRightTaxi(DriveSubsystem drive, AutoPIDControllers autoPIDControllers) {
        this.drive = drive;
        this.autoPIDControllers = autoPIDControllers;

        addRequirements(drive);
    }

    public Command cargorighttaxi(){
        /*
        try{
            Path route1 = Filesystem.getDeployDirectory().toPath().resolve(route1);
            route = TrajectoryUtil.fromPathweaverJson(route1);
        }catch (IOException ex){
        }
        */

        
        TrajectoryConfig config = new TrajectoryConfig(0.6, 0.2).
        setKinematics(Constants.PhysicalConstants.KINEMATICS).
        addConstraint(new SwerveDriveKinematicsConstraint(Constants.PhysicalConstants.KINEMATICS, 14)).
        setEndVelocity(0).
        setReversed(false);

        Trajectory route = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(
                new Translation2d(-0.5, 0)
            ), 
            new Pose2d(-1, 0, Rotation2d.fromDegrees(180)), 
            config
        );
        
        return new SequentialCommandGroup(
            new InstantCommand(drive::reset, drive),
            new InstantCommand(() -> drive.setPose(route.getInitialPose()), drive),
            new SwerveControllerCommand(
                route,
                drive::getPose, 
                Constants.PhysicalConstants.KINEMATICS, 
                new HolonomicDriveController(
                autoPIDControllers.CRITICAL_X(), 
                autoPIDControllers.CRITICAL_Y(), 
                new ProfiledPIDController(
                    autoPIDControllers.CRITICAL_THETA().getP(), 
                    autoPIDControllers.CRITICAL_THETA().getI(), 
                    autoPIDControllers.CRITICAL_THETA().getD(), 
                    new Constraints(0.4, 0.15)
                    )
                ),
                drive::OutputModuleInfo, 
                drive)
        );
    }
    
}
