package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.event.NetworkBooleanEvent;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

    private NetworkTable table;

    public LimelightSubsystem() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public double getX() {
        return table.getEntry("tx").getDouble(0);
    }

    public double getY() {
        return table.getEntry("ty").getDouble(0);
    }

    public double getA() {
        return table.getEntry("ta").getDouble(0);
    }

    public long getId() {
        return table.getEntry("tid").getInteger(0);
    }

}