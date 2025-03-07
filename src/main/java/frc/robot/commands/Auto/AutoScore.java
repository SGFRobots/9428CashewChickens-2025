package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Coral;

public class AutoScore extends Command{
    private Timer timer;
    private Coral mCoral;
    
    public AutoScore(Coral pCoral) {
        // Initialize variables
        timer = new Timer();
        mCoral = pCoral;
    }

    @Override
    public void initialize() {
        // Reset timer value to zero
        timer.restart();
    }
    
    @Override 
    public void execute() {
        // Apply power to the coral shooter
        mCoral.setPower(0.5);
    }

    @Override
    public void end(boolean isFinished) {
        // Stop the coral shooter
        mCoral.stop();
        // Stop the timer
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        // Stop the command after 0.75 seconds
        if (timer.get() > 0.75){
            return true;
        }
        return false;
    }
}
