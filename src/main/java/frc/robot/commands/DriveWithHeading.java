package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveWithHeading extends CommandBase {
    private DriveSubsystem drive;
    private double heading;
    private PIDController turnController;
    private double x1, x2, y;

    public DriveWithHeading(DriveSubsystem drive, double heading){
        this.drive = drive;
        this.heading = heading;

        addRequirements(drive);
    }

    @Override
    public void initialize(){
        turnController = new PIDController(1, 0, 0);
        y = 0;
        if(heading > 180){
            heading -= 360;
        }   
    }

    @Override
    public void execute(){
        // ---------------------------
        // DO SWERVE CALCULATIONS
        // ---------------------------

        double a = x1 - x2 * (Constants.PhysicalConstants.SIDE_LENGTH / Constants.PhysicalConstants.SIDE_TO_CORNER );
        double b = x1 + x2 * (Constants.PhysicalConstants.SIDE_LENGTH / Constants.PhysicalConstants.SIDE_TO_CORNER);
        double c = y - x2 * (Constants.PhysicalConstants.SIDE_WIDTH / Constants.PhysicalConstants.SIDE_TO_CORNER);
        double d = y + x2 * (Constants.PhysicalConstants.SIDE_WIDTH / Constants.PhysicalConstants.SIDE_TO_CORNER);

        double frontleftspeed = Math.sqrt(Math.pow(b, 2) + Math.pow(c, 2));
        double frontrightspeed = Math.sqrt(Math.pow(b, 2) + Math.pow(d, 2));
        double rearleftspeed = Math.sqrt(Math.pow(a, 2) + Math.pow(c, 2));
        double rearrightspeed = Math.sqrt(Math.pow(a, 2) + Math.pow(d, 2));

        double frontleftangle = Math.atan2(b, c) * 180 / Math.PI;
        double frontrightangle = Math.atan2(b, d) * 180 / Math.PI;
        double rearleftangle = Math.atan2(a, c) * 180 / Math.PI;
        double rearrightangle = Math.atan2(a, d) * 180 / Math.PI;

        turnController.enableContinuousInput(-180, 180);

        double headingError = drive.getHeading().getDegrees() - heading;

        double turnPower = turnController.calculate(headingError, heading);

        if(Math.abs(turnPower) > 45){
            turnPower = 45 * Math.signum(turnPower);
        }

        drive.frontLeft.setSpeed(frontleftspeed);
        drive.frontRight.setSpeed(frontrightspeed);
        drive.rearLeft.setSpeed(rearleftspeed);
        drive.rearRight.setSpeed(rearrightspeed);

        drive.frontLeft.setAngle(frontleftangle + turnPower);
        drive.frontRight.setAngle(frontrightspeed + turnPower);
        drive.rearLeft.setAngle(rearleftspeed - turnPower);
        drive.rearRight.setAngle(rearrightspeed - turnPower);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
