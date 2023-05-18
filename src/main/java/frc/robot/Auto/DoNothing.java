package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoSpinUp;
import frc.robot.commands.SpinUp;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Sequencer;
import frc.robot.subsystems.ShooterSubsystem;

public class DoNothing extends CommandBase {
    public DriveSubsystem drive;
    public Sequencer sequencer;
    public ShooterSubsystem shooter;

    public DoNothing(DriveSubsystem drive, ShooterSubsystem shooter, Sequencer sequencer) {
        this.drive = drive;
        this.sequencer = sequencer;
        this.shooter = shooter;

        addRequirements(drive);
    }

    public Command doNothing() {
        return new SequentialCommandGroup(
            new InstantCommand(drive::reset, drive),
            new AutoSpinUp(shooter, sequencer, 1, 6)
        );
    }
}
