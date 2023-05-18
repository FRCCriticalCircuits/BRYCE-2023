package frc.robot.Auto;

import java.util.HashMap;
import java.util.List;

import com.fasterxml.jackson.databind.deser.std.ThrowableDeserializer;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Util.DriveController;
import frc.robot.Util.GoalType.goalType;
import frc.robot.commands.AlignToTarget;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoShootWithVision;
import frc.robot.commands.AutoSpinUp;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Sequencer;
import frc.robot.subsystems.ShooterSubsystem;

public class TwoCargoAndBalanceTaxi extends CommandBase{
    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private Intake intake;
    private LimelightSubsystem limelight;
    private DriveController driveController;
    private Sequencer sequencer;

    public TwoCargoAndBalanceTaxi(DriveSubsystem drive, ShooterSubsystem shooter, Sequencer sequencer, Intake intake, LimelightSubsystem limelight, DriveController driveController) {
        this.drive = drive;
        this.shooter = shooter;
        this.intake = intake;
        this.limelight = limelight;
        this.driveController = driveController;
        this.sequencer = sequencer;

        addRequirements(drive);
    }

    public Command twocargoandbalancetaxi() {
        new PathPlanner();
        List<PathPlannerTrajectory> taxi = PathPlanner.loadPathGroup("PLS Work", 2, 1.5);

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Intake", new AutoIntake(intake, sequencer, 1));

        SwerveAutoBuilder autoBuilder 
        = new SwerveAutoBuilder(
            drive::getPose, 
            drive::setPose, 
            new PIDConstants(
                0.85, 
                0, 
                0
            ), 
            new PIDConstants(
                -0.4, 
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
            new AutoSpinUp(shooter, sequencer, 1, 10),
            autoBuilder.fullAuto(taxi),
            new ParallelDeadlineGroup(
                new WaitCommand(4),
                new AutoBalance(drive, driveController, true)
            ), new SequentialCommandGroup(
                    new ParallelDeadlineGroup(
                        new WaitCommand(1),
                        new AutoShootWithVision(shooter, sequencer, limelight, goalType.MID, 1)
                    )
                ),
            new InstantCommand(() -> drive.setGyroOffset(180), drive)
        );
    }
}