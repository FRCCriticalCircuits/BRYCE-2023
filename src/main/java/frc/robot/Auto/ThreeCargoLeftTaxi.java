package frc.robot.Auto;

import java.security.InvalidKeyException;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Util.AutoPIDControllers;
import frc.robot.Util.DriveController;
import frc.robot.Util.GoalType.goalType;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoShootWithVision;
import frc.robot.commands.AutoSpinUp;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Sequencer;
import frc.robot.subsystems.ShooterSubsystem;

public class ThreeCargoLeftTaxi extends CommandBase {
    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private Intake intake;
    private Sequencer sequencer; 
    private LimelightSubsystem limelight;
    private DriveController drivecontroller;
    private AutoPIDControllers autopidControllers;

    public ThreeCargoLeftTaxi(DriveSubsystem drive, ShooterSubsystem shooter, Intake intake, Sequencer sequencer, LimelightSubsystem limelight, DriveController drivecontroller, AutoPIDControllers autoPIDControllers) {
        this.drive = drive;
        this.shooter = shooter;
        this.intake = intake;
        this.sequencer = sequencer;
        this.limelight = limelight;
        this.drivecontroller = drivecontroller;
        this.autopidControllers = autoPIDControllers;

        addRequirements(drive);
    }


    public Command threecargolefttaxi() {
        List<PathPlannerTrajectory> leftTaxi = PathPlanner.loadPathGroup("Left Taxi 3 Cargo", 3, 2.4, true);

        HashMap<String, Command> eventMap = new HashMap<>();

        eventMap.put("MARKER 1", new PrintCommand("Marker 1 Passed"));
        eventMap.put("Intake", new AutoIntake(intake, sequencer, 1));
        eventMap.put("Shoot", new AutoShootWithVision(shooter, sequencer, limelight, goalType.MID, 1));
        

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
            true, 
            drive
        );

        return new SequentialCommandGroup(
            new InstantCommand(drive::reset, drive),
            new AutoSpinUp(shooter, sequencer, 1, 8),
            autoBuilder.fullAuto(leftTaxi),
            new AutoBalanceBackup(drive, drivecontroller),
            new InstantCommand(drive::resetHeading, drive)
        );
    }
}
