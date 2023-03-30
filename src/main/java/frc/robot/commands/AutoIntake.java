package frc.robot.commands;

import javax.sound.midi.Sequence;

import edu.wpi.first.hal.simulation.DIODataJNI;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Sequencer;

public class AutoIntake extends CommandBase{
    private Intake intake;
    private Sequencer sequencer;
    private double time = 1;
    private boolean timeOver = false;

    public AutoIntake(Intake intake, Sequencer sequencer) {
        this.intake = intake;
        this.sequencer = sequencer;

        addRequirements(intake);
    }

    public AutoIntake(Intake intake, Sequencer sequencer, double time) {
        this.intake = intake;
        this.sequencer = sequencer;
        this.time = time;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        new Thread(
            () -> {
                try {
                    Thread.sleep((int) time * 1000);
                    timeOver = true;
                } catch (Exception e) {
                }
            }
        ).start();
    }

    @Override

    public void execute() {
        intake.runIntake();
    }

    @Override
    public void end(boolean Interrupted) {
        intake.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
