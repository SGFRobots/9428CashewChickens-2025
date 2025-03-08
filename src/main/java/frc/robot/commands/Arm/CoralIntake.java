package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Coral;

public class CoralIntake extends Command{
    private Timer timer;
    private Coral mCoral;
    private Algae mAlgae;
    
    public CoralIntake(Coral pCoral, Algae pAlgae) {
        // Initialize variables
        timer = new Timer();
        mCoral = pCoral;
        mAlgae = pAlgae;
    }

    @Override
    public void initialize() {
        // Reset timer value to zero
        timer.restart();
    }
    
    @Override 
    public void execute() {
        // Apply power to the coral shooter
        mAlgae.setDesiredPos(1);
        mCoral.setPower(0.1);
    }

    @Override
    public void end(boolean isFinished) {
        // Stop the coral shooter
        mCoral.stop();
        // Stop the timer
        timer.stop();
        mAlgae.setDesiredPos(0);
    }

    @Override
    public boolean isFinished() {
        // Stop the command after 0.75 seconds
        if (timer.get() > 2){
            return true;
        }
        return false;
    }
}
