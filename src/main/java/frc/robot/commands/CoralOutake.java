package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Coral;


public class CoralOutake extends Command{
    private final Coral mCoral;

    public CoralOutake(Coral pCoral) {
        mCoral = pCoral;
    }

    @Override
    public void initialize() {
        mCoral.coralIn = true;
    }

    @Override 
    public void execute() {
        mCoral.setPower(.5);
    }
    
    @Override
    public void end(boolean isFinished) {
        mCoral.stop();
    }
    
    @Override
    public boolean isFinished() {
        if (mCoral.getOutSensorDist() > 500){
            mCoral.coralIn = false;
            return true;
        }
        return false;
    }
}
