package frc.robot.Auto;

import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Util.AutoPIDControllers;
import frc.robot.subsystems.DriveController;
import frc.robot.subsystems.DriveSubsystem;

public class TwoCargoLeftTaxi extends CommandBase {
    public DriveSubsystem drive;
    public DriveController drivecontroller;
    public AutoPIDControllers autopidControllers;

    public TwoCargoLeftTaxi(DriveSubsystem drive, DriveController drivecontroller, AutoPIDControllers autoPIDControllers) {
        this.drive = drive;
        this.drivecontroller = drivecontroller;
        this.autopidControllers = autoPIDControllers;

        addRequirements(drive);
    }


    public Command twocargolefttaxi() {
        TrajectoryConfig config = new TrajectoryConfig(1, 0.5).
        setKinematics(Constants.PhysicalConstants.KINEMATICS).
        addConstraint(new SwerveDriveKinematicsConstraint(Constants.PhysicalConstants.KINEMATICS, 1))
        .setReversed(false).setEndVelocity(0);

        Trajectory route = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)), 
        List.of(
        ), 
        new Pose2d(3, 0, Rotation2d.fromDegrees(180)), 
        config
        );

        return new SequentialCommandGroup(
            new InstantCommand(
                drive::reset, 
                drive
            ), 
            new SwerveControllerCommand(
                route, 
                drive::getPose, 
                Constants.PhysicalConstants.KINEMATICS,
                new HolonomicDriveController(
                    autopidControllers.CRITICAL_X(),
                    autopidControllers.CRITICAL_Y(),
                    new ProfiledPIDController(
                        autopidControllers.CRITICAL_THETA().getP(), 
                        autopidControllers.CRITICAL_THETA().getI(), 
                        autopidControllers.CRITICAL_THETA().getD(), 
                        new Constraints(1, 0.5)
                    )
                ),
                drive::getHeading,
                drive::OutputModuleInfo,
                drive).andThen(drive::stop)
                );
    }

}
