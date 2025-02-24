package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.Algae;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeIntake extends Command {
    private final Algae mAlgae;
    private Timer timer;
    public AlgaeIntake(Algae pAlgae) {
        mAlgae = pAlgae;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override 
    public void execute() {
        timer.start();
        mAlgae.setPosPower(0.10);
    }

    @Override
    public void end(boolean isFinished) {
        mAlgae.stop();
    }

    @Override
    public boolean isFinished() {
        if (timer.get() > 0.5){
            return true;
        }
        return false;
    }
}