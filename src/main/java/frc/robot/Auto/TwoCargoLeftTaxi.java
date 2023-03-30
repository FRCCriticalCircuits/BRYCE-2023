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
import frc.robot.Util.AutoPIDControllers;
import frc.robot.Util.DriveController;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoSpinUp;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Sequencer;
import frc.robot.subsystems.ShooterSubsystem;

public class TwoCargoLeftTaxi extends CommandBase {
    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private Intake intake;
    private Sequencer sequencer; 
    private DriveController drivecontroller;
    private AutoPIDControllers autopidControllers;

    public TwoCargoLeftTaxi(DriveSubsystem drive, ShooterSubsystem shooter, Intake intake, Sequencer sequencer, DriveController drivecontroller, AutoPIDControllers autoPIDControllers) {
        this.drive = drive;
        this.shooter = shooter;
        this.intake = intake;
        this.sequencer = sequencer;
        this.drivecontroller = drivecontroller;
        this.autopidControllers = autoPIDControllers;

        addRequirements(drive);
    }


    public Command twocargolefttaxi() {
        List<PathPlannerTrajectory> leftTaxi = PathPlanner.loadPathGroup("Left Taxi", 1.2, 0.7);

        HashMap<String, Command> eventMap = new HashMap<>();

        eventMap.put("MARKER 1", new PrintCommand("Marker 1 Passed"));
        eventMap.put("IntakeStart", new AutoIntake(intake, sequencer, 2));
        eventMap.put("ShootSecond", new AutoSpinUp(shooter, sequencer, 0, 1.5, 13));

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
            true, 
            drive
        );

        return new SequentialCommandGroup(
            new AutoSpinUp(shooter, sequencer, 0, 1.5, 11),
            autoBuilder.fullAuto(leftTaxi),
            new InstantCommand(drive::resetHeading, drive)
        );
    }
}
