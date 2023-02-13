package frc.robot.Auto;

import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PIDContollers;

public class DriveStraight {
    DriveSubsystem drive;
    PIDContollers pidContollers;

    public DriveStraight(DriveSubsystem drive, PIDContollers pidContollers){
        this.drive = drive;
        this.pidContollers = pidContollers;
    }

    public Command driveStaight() {

        TrajectoryConfig config = new TrajectoryConfig(
            2, 
            1).setKinematics(
                new SwerveDriveKinematics(
                    new Translation2d(Units.inchesToMeters(Constants.PhysicalConstants.SIDE_WIDTH / 2), Units.inchesToMeters(Constants.PhysicalConstants.SIDE_LENGTH / 2)),
                    new Translation2d(-Units.inchesToMeters(Constants.PhysicalConstants.SIDE_WIDTH / 2), Units.inchesToMeters(Constants.PhysicalConstants.SIDE_LENGTH / 2)),
                    new Translation2d(Units.inchesToMeters(Constants.PhysicalConstants.SIDE_WIDTH / 2), -Units.inchesToMeters(Constants.PhysicalConstants.SIDE_LENGTH / 2)),
                    new Translation2d(-Units.inchesToMeters(Constants.PhysicalConstants.SIDE_WIDTH / 2), -Units.inchesToMeters(Constants.PhysicalConstants.SIDE_LENGTH / 2))
                )
            ).addConstraint(new SwerveDriveKinematicsConstraint(
                new SwerveDriveKinematics(
                    new Translation2d(Units.inchesToMeters(Constants.PhysicalConstants.SIDE_WIDTH / 2), Units.inchesToMeters(Constants.PhysicalConstants.SIDE_LENGTH / 2)),
                    new Translation2d(-Units.inchesToMeters(Constants.PhysicalConstants.SIDE_WIDTH / 2), Units.inchesToMeters(Constants.PhysicalConstants.SIDE_LENGTH / 2)),
                    new Translation2d(Units.inchesToMeters(Constants.PhysicalConstants.SIDE_WIDTH / 2), -Units.inchesToMeters(Constants.PhysicalConstants.SIDE_LENGTH / 2)),
                    new Translation2d(-Units.inchesToMeters(Constants.PhysicalConstants.SIDE_WIDTH / 2), -Units.inchesToMeters(Constants.PhysicalConstants.SIDE_LENGTH / 2))
                ), 
                10)
            ).setEndVelocity(0);


        Trajectory route = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(), 
            new Pose2d(3, 0, new Rotation2d(0)), config
        );
        
        return new SequentialCommandGroup(
            new InstantCommand(drive::reset, drive),
            new SwerveControllerCommand(route, drive::getPose, new SwerveDriveKinematics(
                new Translation2d(Units.inchesToMeters(Constants.PhysicalConstants.SIDE_WIDTH / 2), Units.inchesToMeters(Constants.PhysicalConstants.SIDE_LENGTH / 2)),
                new Translation2d(-Units.inchesToMeters(Constants.PhysicalConstants.SIDE_WIDTH / 2), Units.inchesToMeters(Constants.PhysicalConstants.SIDE_LENGTH / 2)),
                new Translation2d(Units.inchesToMeters(Constants.PhysicalConstants.SIDE_WIDTH / 2), -Units.inchesToMeters(Constants.PhysicalConstants.SIDE_LENGTH / 2)),
                new Translation2d(-Units.inchesToMeters(Constants.PhysicalConstants.SIDE_WIDTH / 2), -Units.inchesToMeters(Constants.PhysicalConstants.SIDE_LENGTH / 2))
                ), 
                new HolonomicDriveController(
                    pidContollers.CRITICAL_X(), 
                    pidContollers.CRITICAL_Y(), 
                    new ProfiledPIDController(
                        pidContollers.CRITICAL_THETA().getP(),
                        pidContollers.CRITICAL_THETA().getI(),
                        pidContollers.CRITICAL_THETA().getD(),
                        new Constraints(3, 1)
                    )
                ),
                drive::OutputModuleInfo, 
                drive
            ).andThen(drive::stop)
        );
    }
}
