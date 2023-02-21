package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinUp extends CommandBase {
    public ShooterSubsystem shooter;
    public double velocity;

    public SpinUp(ShooterSubsystem shooter, double velocity) {
        this.shooter = shooter;
        this.velocity = velocity;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setSpeed(40);
    }
}
