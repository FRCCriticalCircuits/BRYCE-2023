package frc.robot.Auto;

import java.util.List;

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Util.AutoPIDControllers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Sequencer;
import frc.robot.subsystems.ShooterSubsystem;

public class MiddleAuto extends CommandBase{
    DriveSubsystem drive;
    Sequencer sequencer;
    ShooterSubsystem shooter;
    AutoPIDControllers autoPIDControllers;
    //String route1 = "frc/paths/route1.wpilib.json";
    //Trajectory route = new Trajectory();

    public MiddleAuto(DriveSubsystem drive, ShooterSubsystem shooter, Sequencer sequencer,AutoPIDControllers autoPIDControllers) {
        this.drive = drive;
        this.autoPIDControllers = autoPIDControllers;
        this.sequencer = sequencer;
        this.shooter = shooter;

        addRequirements(drive);
    }

    public Command middletaxi(){
        /*
        try{
            Path route1 = Filesystem.getDeployDirectory().toPath().resolve(route1);
            route = TrajectoryUtil.fromPathweaverJson(route1);
        }catch (IOException ex){
        }
        */
        drive.reset();

        drive.setBrakeMode(true);
        
        TrajectoryConfig config = new TrajectoryConfig(4, 2).
        setKinematics(Constants.PhysicalConstants.KINEMATICS).
        addConstraint(new SwerveDriveKinematicsConstraint(Constants.PhysicalConstants.KINEMATICS, Constants.PhysicalConstants.MAX_METERS_PER_SECOND)).
        setReversed(false);

        Trajectory route = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(
                //new Translation2d(0.5, 0)
            ), 
            new Pose2d(17, 0, Rotation2d.fromDegrees(0)), 
            config
        );
        
        return new SequentialCommandGroup(
            //new InstantCommand(drive::reset, drive),
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
                        new Constraints(2, 0.4)
                    )
                ),
                drive::getRotation2d,
                drive::OutputModuleInfo, 
                drive).andThen(drive::baselock, drive)
        );
    }
    
}
